package main

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"strconv"
	"strings"
)

const (
	assertionHeader = iota
	assertionLineNo
	assertionFunction
)
const (
	header = iota
	register0
	register1
	register2
	register3
	returnControl
	returnAddress
	faultAddress
)

func main() {
	mapFile := flag.String("mapFile", "", "Path to .map file")
	m122 := flag.String("m122", "-", "Path to file with M122 output (even partial) or \"-\" for stdin")
	flag.Parse()

	if *m122 == "" {
		log.Fatal("-m122 needs to be provided")
	}
	if *mapFile == "" {
		log.Fatal("-mapFile needs to be provided")
	}

	var r io.Reader
	if *m122 == "-" {
		r = bufio.NewReader(os.Stdin)
	} else {
		f, err := os.Open(*m122)
		if err != nil {
			log.Fatal(err)
		}
		defer f.Close()
		r = f
	}

	var err error
	faultedAddress := uint64(0)
	scanner := bufio.NewScanner(r)
	isAssertion := false
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())

		if line == "" {
			continue
		}
		if strings.HasPrefix(line, "Last software reset") {
			isAssertion = strings.Contains(line, "reason: Assertion")
			continue
		}
		if strings.HasPrefix(line, "Stack:") {
			stackElems := strings.Fields(line)
			var faultElem string
			if isAssertion {
				faultElem = stackElems[assertionFunction]
			} else {
				faultElem = stackElems[faultAddress]
			}

			// Get the fault address
			faultedAddress, err = strconv.ParseUint(faultElem, 16, 64)
			if err != nil {
				log.Fatal(err)
			}

			faultLocation, lastAddress, err := getFailingBlock(*mapFile, faultedAddress)
			if err != nil {
				log.Fatal(err)
			}

			fmt.Println("Faulted at:")
			for _, l := range faultLocation {
				fmt.Println(l)
			}
			if isAssertion {
				lineNo, _ := strconv.ParseUint(stackElems[assertionLineNo], 16, 64)
				fmt.Printf("Error is at line: %d\n", lineNo)
			} else {
				fmt.Printf("Error is at offset: %#x\n", (faultedAddress - lastAddress))
			}
		}
	}
}

func getFailingBlock(mapFile string, faultedAddress uint64) ([]string, uint64, error) {
	// Open the map file
	mf, err := os.Open(mapFile)
	if err != nil {
		return nil, 0, err
	}
	defer mf.Close()
	mapScanner := bufio.NewScanner(mf)

	var faultLocation []string
	var prevFaultFunction string
	var faultFunction string
	lastAddress := uint64(0)
	foundStart := false
	addressOfLine := uint64(0)
	for mapScanner.Scan() {
		mapLine := mapScanner.Text()

		// Advance to the interesting section
		if !foundStart {
			if !strings.HasPrefix(mapLine, ".text") {
				continue
			}
			foundStart = true
		}
		fields := strings.Fields(mapLine)

		// Lines with only one field indicate the next function
		if len(fields) < 2 {
			prevFaultFunction = faultFunction
			faultFunction = fields[0]
			continue
		}

		// Get the address of the current line
		var addressField string
		for _, f := range fields {
			if strings.HasPrefix(f, "0x") {
				addressField = f[2:]
				break
			}
		}

		// Still no address
		if addressField == "" {
			continue
		}

		addressOfLine, err = strconv.ParseUint(addressField, 16, 64)
		if err != nil {
			return nil, 0, errors.New(fmt.Sprintf("Could not parse line: %s", mapLine))
		}

		if addressOfLine <= faultedAddress {

			// Reset the slice of lines if we see a new address
			if addressOfLine != lastAddress {
				lastAddress = addressOfLine
				faultLocation = make([]string, 0, 0)
				faultLocation = append(faultLocation, prevFaultFunction)
			}
			faultLocation = append(faultLocation, mapLine)
		} else {
			// We reached the function after our fault
			break
		}
	}

	return faultLocation, lastAddress, nil
}
