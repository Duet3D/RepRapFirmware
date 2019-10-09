package main

import (
	"encoding/binary"
	"hash/crc32"
	"io/ioutil"
	"os"
)

func main() {

	// Loop through arguments
	for _, f := range os.Args[1:] {

		// Read the file to memory and panic if it fails
		b, err := ioutil.ReadFile(f)
		if err != nil {
			panic(err)
		}
		// Calculate CRC32 with IEEE polynomials
		c := crc32.ChecksumIEEE(b)

		// Open the same file for appending and panic if it fails
		f, err := os.OpenFile(f, os.O_APPEND|os.O_WRONLY, 0644)
		if err != nil {
			panic(err)
		}
		defer f.Close()

		// Create little-endian represenation of CRC32 sum
		le := make([]byte, crc32.Size)
		binary.LittleEndian.PutUint32(le, c)

		// Append the CRC32 and panic if it fails
		if _, err = f.Write(le); err != nil {
			panic(err)
		}
	}
}
