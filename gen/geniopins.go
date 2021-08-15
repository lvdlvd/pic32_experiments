/*
	Geniopins reads a pin definition file on stdin,
	validates and outputs C code

 lines beginning with '#' are ignored

   type:
      A   analog input
      I   digital input
      IPU    with pullup
      IPD    with pulldonwn
      O   output
      OD     with open drain

   <n>  XXXXX
   		only checks if XXXXX is a valid name for pin n.

   <n>  RPxy   <peripheral>  I/O[D]
      generates code to map the peripheral to the pin

   <n>  R[P]Xy  Xy  I[PU/D]  Alias
      generates code to configure pin Xy for input
      also generates a X_ALIAS mask for use when reading PORTX

   <n>  R[P]Xy  Xy  O[D]  Alias [State]
      generates code to configure pin Xy for output, and optionally set to state
      also generates a X_ALIAS mask for use when writing LATX[SET/CLR/INV]

   <n>  R[P]Xy  ANyy  A  Alias
      generates code to configure pin ANyy for analog input, plus
      code to TODO use ADC....

*/
package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"os"
	"regexp"
	"strconv"
	"strings"
)

//go:generate stringer -output=strings.go -type=PinType

var (
	fPIC = flag.String("PIC", "32MK1024GPE100", "which model to target.")
)

var validpins = map[string][]string{
	"32MK0512GPE064": PIC32MKxxxxMCF064,
	"32MK0512GPE100": PIC32MKxxxxMCF100,
	"32MK0512MCF064": PIC32MKxxxxGPE064,
	"32MK0512MCF100": PIC32MKxxxxGPE100,
	"32MK1024GPE064": PIC32MKxxxxMCF064,
	"32MK1024GPE100": PIC32MKxxxxMCF100,
	"32MK1024MCF064": PIC32MKxxxxGPE064,
	"32MK1024MCF100": PIC32MKxxxxGPE100,
}

type PinType uint32

const (
	INVALID PinType = iota
	ANALOGIN
	ANALOGOUT
	DIGITALIN
	DIGITALINPULLUP
	DIGITALINPULLDOWN
	DIGITALOUT
	DIGITALOUTOPENDRAIN
)

func parsePinType(s string) PinType {
	switch s {
	case "A":
		return ANALOGIN
	case "AO":
		return ANALOGOUT
	case "I":
		return DIGITALIN
	case "IPU":
		return DIGITALINPULLUP
	case "IPD":
		return DIGITALINPULLDOWN
	case "O":
		return DIGITALOUT
	case "OD":
		return DIGITALOUTOPENDRAIN
	}
	return INVALID
}

type PinDef struct {
	Pin         int
	Name        string
	Signal      string
	Type        PinType
	Alias       string
	Initial     bool
	InitialHigh bool
}

var rePortBit = regexp.MustCompile(`RP?([A-G])(\d+)`)

func (d *PinDef) Port() string {
	parts := rePortBit.FindStringSubmatch(d.Name)
	if len(parts) != 3 {
		return ""
	}
	return parts[1]
}
func (d *PinDef) Bit() int {
	parts := rePortBit.FindStringSubmatch(d.Name)
	if len(parts) != 3 {
		return -1
	}
	n, _ := strconv.Atoi(parts[2])
	return n
}

func (d *PinDef) IsGPIO() bool {
	parts := rePortBit.FindStringSubmatch(d.Name)
	if len(parts) != 3 {
		return false
	}
	return d.Signal == parts[1]+parts[2]
}

func (d *PinDef) GPIO() string {
	parts := rePortBit.FindStringSubmatch(d.Name)
	if len(parts) != 3 {
		return ""
	}
	return parts[1] + parts[2]
}

var portstate ['G' - 'A' + 1]struct {
	Analog    uint16
	Tristate  uint16
	Latch     uint16 // initial state
	OpenDrain uint16
	Pullup    uint16
	Pulldown  uint16
}

func contains(i int, set []int) bool {
	for _, v := range set {
		if i == v {
			return true
		}
	}
	return false
}

func main() {

	flag.Parse()

	if validpins[*fPIC] == nil {
		log.Fatalf("Invalid PIC model specified: %q", *fPIC)
	}
	vp := map[string][]int{}
	for i, v := range validpins[*fPIC] {
		for _, vv := range strings.Split(v, "/") {
			vp[vv] = append(vp[vv], i+1)
		}
	}

	remap := map[string]map[string]uint{}
	for _, v := range imux {
		for _, vv := range strings.Fields(v.signals) {
			remap[vv] = v.pins
		}
	}
	for _, v := range omux {
		for _, vv := range strings.Fields(v.pins) {
			remap[vv] = v.signals
		}
	}

	errs := 0
	var defs []*PinDef

	scanner := bufio.NewScanner(os.Stdin)
	for ln := 1; scanner.Scan(); ln++ {
		line := scanner.Text()
		if idx := strings.Index(line, "#"); idx >= 0 {
			line = line[:idx]
		}
		line = strings.TrimSpace(line)

		if line == "" {
			continue
		}

		fields := strings.Fields(line)
		if len(fields) < 2 {
			log.Printf("line %d: invalid pin declaration %q", ln, line)
			errs++
			continue
		}
		pin, err := strconv.Atoi(fields[0])
		if err != nil {
			log.Printf("line %d: invalid pin number %q %v", ln, line, err)
			errs++
			continue
		}
		def := &PinDef{
			Pin:  pin,
			Name: fields[1],
		}

		if n, ok := vp[def.Name]; !ok || !contains(def.Pin, n) {
			log.Printf("line %d:%q not a valid name for pin %d", ln, def.Name, def.Pin)
			if ok {
				log.Printf("line %d: (%q is a valid name for pins %d)", ln, def.Name, n)
			}
			errs++
			continue
		}
		if len(fields) >= 3 {
			def.Signal = fields[2]
		}
		if len(fields) >= 4 {
			def.Type = parsePinType(fields[3])
		}
		if len(fields) >= 5 {
			def.Alias = fields[4]
		}
		if len(fields) >= 6 {

			switch def.Type {
			case DIGITALOUT, DIGITALOUTOPENDRAIN:

			default:
				log.Printf("line %d: initial state is only valid for type O[D], not %q", ln, fields[4])
			}

			def.Initial = true
			switch fields[5] {
			case "LOW":
				def.InitialHigh = false
			case "HIGH":
				def.InitialHigh = true
			default:
				log.Printf("line %d: invalid initial state %q (use LOW or HIGH)", ln, fields[5])
				errs++
				continue
			}
		}

		if !def.IsGPIO() {
			switch def.Type {
			case DIGITALOUT, DIGITALOUTOPENDRAIN:
				if _, ok := remap[def.GPIO()][def.Signal]; !ok {
					log.Printf("line %d: no mapping of %q to %q", ln, def.Signal, def.GPIO())
					if contains(def.Pin, vp[def.Signal]) {
						log.Printf("line %d: (however %q is a valid name for pin %d without remapping)", ln, def.Signal, def.Pin)
					}
					errs++
				}

			case DIGITALIN, DIGITALINPULLUP, DIGITALINPULLDOWN:
				if _, ok := remap[def.Signal][def.GPIO()]; !ok {
					log.Printf("line %d: no mapping of %q to %q", ln, def.GPIO(), def.Signal)
					errs++
					if contains(def.Pin, vp[def.Signal]) {
						log.Printf("line %d: (however %q is a valid name for pin %d without remapping)", ln, def.Signal, def.Pin)
					}
				}

			}
		}

		defs = append(defs, def)

	}
	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
	if errs > 0 {
		log.Fatalf("%d errors. Please fix and re-run.", errs)
	}

	portused := map[byte]bool{}

	for _, def := range defs {
		//fmt.Println(*def, def.Port(), def.Bit(), def.IsGPIO())

		bit := def.Bit()
		if bit < 0 {
			continue
		}
		port := def.Port()[0] - 'A'
		portused[port] = true
		switch def.Type {
		case ANALOGIN:
			portstate[port].Analog |= 1 << bit
		case DIGITALIN:
		case DIGITALINPULLUP:
			portstate[port].Pullup |= 1 << bit
		case DIGITALINPULLDOWN:
			portstate[port].Pulldown |= 1 << bit
		case DIGITALOUTOPENDRAIN:
			portstate[port].OpenDrain |= 1 << bit
			fallthrough
		case DIGITALOUT:
			portstate[port].Tristate |= 1 << bit // note: inverse!
			if def.Initial && def.InitialHigh {
				portstate[port].Latch |= 1 << bit
			}
		}
	}

	fmt.Printf("// Code generated by geniopins for PIC%s -- DO NOT EDIT\n\n", *fPIC)
	fmt.Println("// port initialisation")
	fmt.Println("inline void initIOPins(void) {")

	for port := byte(0); port <= 'G'-'A'; port++ {
		if !portused[port] {
			continue
		}
		fmt.Printf("\tANSEL%c   \t= 0b%016b;\n", 'A'+port, portstate[port].Analog)
		if v := portstate[port].Pullup; v != 0 {
			fmt.Printf("\tCNPU%cSET\t= 0b%016b;\n", 'A'+port, v)
		}
		if v := portstate[port].Pulldown; v != 0 {
			fmt.Printf("\tCNPD%cSET\t= 0b%016b;\n", 'A'+port, v)
		}
		if v := portstate[port].OpenDrain; v != 0 {
			fmt.Printf("\tODC%cSET  \t= 0b%016b;\n", 'A'+port, v)
		}
		if v := portstate[port].Latch; v != 0 {
			fmt.Printf("\tLAT%cSET  \t= 0b%016b;\n", 'A'+port, v)
		}
		if v := portstate[port].Tristate; v != 0 {
			fmt.Printf("\tTRIS%cCLR \t= 0b%016b;\n", 'A'+port, v)
		} // note, inverse!
		fmt.Println()
	}

	fmt.Println("\t// input peripheral->pin mappings")

	for _, def := range defs {
		if def.IsGPIO() {
			continue
		}
		switch def.Type {
		case DIGITALIN, DIGITALINPULLUP, DIGITALINPULLDOWN:
			v := remap[def.Signal][def.GPIO()]
			fmt.Printf("\t%sR\t= 0b%04b;\t// %s\n", def.Signal, v, def.GPIO())

		}
	}

	fmt.Println("\n\t// output pin<-peripheral mappings")

	for _, def := range defs {
		if def.IsGPIO() {
			continue
		}
		switch def.Type {
		case DIGITALOUT, DIGITALOUTOPENDRAIN:
			v := remap[def.GPIO()][def.Signal]
			fmt.Printf("\tRP%sR\t= 0b%05b;\t// %s\n", def.GPIO(), v, def.Signal)
		}
	}

	fmt.Println("}")

	fmt.Println("\n// masks for PORTx and LATx[SET|CLR|INV]")

	fmt.Print("enum {")
	for _, def := range defs {
		if def.Alias != "" {
			fmt.Printf("\n\t%s_%s\t= 0x%04x,\t// %s", def.Port(), strings.ToUpper(def.Alias), 1<<def.Bit(), def.GPIO())
		}
	}
	fmt.Println("\n};")
}
