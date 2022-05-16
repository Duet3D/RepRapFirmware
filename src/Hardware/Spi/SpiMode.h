/*
 * SpiMode.h
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SPIMODE_H_
#define SRC_HARDWARE_SPI_SPIMODE_H_

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};

// Definitions for backwards compatibility with RRF code
constexpr SpiMode SPI_MODE_0 = SpiMode::mode0;
constexpr SpiMode SPI_MODE_1 = SpiMode::mode1;
constexpr SpiMode SPI_MODE_2 = SpiMode::mode2;
constexpr SpiMode SPI_MODE_3 = SpiMode::mode3;

#endif /* SRC_HARDWARE_SPI_SPIMODE_H_ */
