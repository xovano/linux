// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD463X SPI ADC driver
 *
 * Copyright 2021 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>

/* Register addresses */
#define AD463X_REG_INTERFACE_CONFIG_A	0x00
#define AD463X_REG_INTERFACE_CONFIG_B	0x01
#define AD463X_REG_DEVICE_CONFIG	0x02
#define AD463X_REG_CHIP_TYPE		0x03
#define AD463X_REG_PRODUCT_ID_L		0x04
#define AD463X_REG_PRODUCT_ID_H		0x05
#define AD463X_REG_CHIP_GRADE		0x06
#define AD463X_REG_SCRATCH_PAD		0x0A
#define AD463X_REG_SPI_REVISION		0x0B
#define AD463X_REG_VENDOR_L		0x0C
#define AD463X_REG_VENDOR_H		0x0D
#define AD463X_REG_STREAM_MODE		0x0E
#define AD463X_REG_EXIT_CFG_MODE	0x14
#define AD463X_REG_AVG			0x15
#define AD463X_REG_OFFSET_BASE		0x16
#define AD463X_REG_OFFSET_X0_0		0x16
#define AD463X_REG_OFFSET_X0_1		0x17
#define AD463X_REG_OFFSET_X0_2		0x18
#define AD463X_REG_OFFSET_X1_0		0x19
#define AD463X_REG_OFFSET_X1_1		0x1A
#define AD463X_REG_OFFSET_X1_2		0x1B
#define AD463X_REG_GAIN_BASE		0x1C
#define AD463X_REG_GAIN_X0_LSB		0x1C
#define AD463X_REG_GAIN_X0_MSB		0x1D
#define AD463X_REG_GAIN_X1_LSB		0x1E
#define AD463X_REG_GAIN_X1_MSB		0x1F
#define AD463X_REG_MODES		0x20
#define AD463X_REG_OSCILATOR		0x21
#define AD463X_REG_IO			0x22
#define AD463X_REG_PAT0			0x23
#define AD463X_REG_PAT1			0x24
#define AD463X_REG_PAT2			0x25
#define AD463X_REG_PAT3			0x26
#define AD463X_REG_DIG_DIAG		0x34
#define AD463X_REG_DIG_ERR		0x35
/* MODES */
#define AD463X_LANE_MODE_MSK		GENMASK(7, 6)
#define AD463X_CLK_MODE_MSK		GENMASK(5, 4)
#define AD463X_DATA_RATE_MODE_MSK	BIT(3)
#define AD463X_OUT_DATA_MODE_MSK	GENMASK(2, 0)
/* EXIT_CFG_MD */
#define AD463X_EXIT_CFG_MODE		BIT(0)

#define AD463X_CONFIG_TIMING		0x2000
#define AD463X_REG_READ_DUMMY		0x00
#define AD463X_REG_WRITE_MASK(x)	(x & 0x7FFF)
#define AD463X_REG_READ_MASK(x)		(x | BIT(15))

#define AD463X_SPI_REG_ACCESS_SPEED	40000000UL

enum ad463x_id {
	ID_AD4630_24,
	ID_AD4630_20,
	ID_AD4630_16,
	ID_AD4631_24,
	ID_AD4631_20,
	ID_AD4631_16,
	ID_AD4632_24,
	ID_AD4632_20,
	ID_AD4632_16,
};

enum ad463x_lane_mode {
	AD463X_ONE_LANE_PER_CH	= 0x00,
	AD463X_TWO_LANES_PER_CH	= BIT(6),
	AD463X_FOUR_LANES_PER_CH = BIT(7),
	AD463X_SHARED_TWO_CH = (BIT(6) | BIT(7)),
};

enum ad463x_clock_mode {
	AD463X_SPI_COMPATIBLE_MODE = 0x00,
	AD463X_ECHO_CLOCK_MODE = BIT(4),
	AD463X_CLOCK_MASTER_MODE = BIT(5),
};

enum ad463x_data_rate_mode {
	AD463X_SINGLE_DATA_RATE = 0x00,
	AD463X_DUAL_DATA_RATE = BIT(3),
};

enum ad463x_out_data_mode {
	AD463X_24_DIFF = 0x00,
	AD463X_16_DIFF_8_COM = 0x01,
	AD463X_24_DIFF_8_COM = 0x02,
	AD463X_30_AVERAGED_DIFF = 0x03,
	AD463X_32_PATTERN = 0x04
};

static const char * const ad463x_lane_mdoes[] = {
	[AD463X_SHARED_TWO_CH] = "one-lane-shared",
	[AD463X_ONE_LANE_PER_CH] = "one-lane-per-ch",
	[AD463X_TWO_LANES_PER_CH] = "two-lanes-per-ch",
	[AD463X_FOUR_LANES_PER_CH] = "four-lanes-per-ch",
};

static const char * const ad463x_clock_mdoes[] = {
	[AD463X_SPI_COMPATIBLE_MODE] = "spi-compatible",
	[AD463X_ECHO_CLOCK_MODE] = "echo-clock",
	[AD463X_CLOCK_MASTER_MODE] = "clock-master",
};

static const char * const ad463x_data_rates[] = {
	[AD463X_SINGLE_DATA_RATE] = "single",
	[AD463X_DUAL_DATA_RATE] = "dual"
};

static const char * const ad463x_out_data_modes[] = {
	[AD463X_16_DIFF_8_COM] = "16diff-8com",
	[AD463X_24_DIFF] = "24diff",
	[AD463X_24_DIFF_8_COM] = "24diff-8com",
	[AD463X_30_AVERAGED_DIFF] = "30diff-avg",
	[AD463X_32_PATTERN] = "32pat"
};

static const unsigned int ad463x_data_widths[] = {
	[AD463X_16_DIFF_8_COM] = 24,
	[AD463X_24_DIFF] = 24,
	[AD463X_24_DIFF_8_COM] = 32,
	[AD463X_30_AVERAGED_DIFF] = 30,
	[AD463X_32_PATTERN] = 32
};

struct ad463x_phy_config {
	enum ad463x_data_rate_mode	data_rate_mode;
	enum ad463x_out_data_mode	out_data_mode;
	enum ad463x_clock_mode		clock_mode;
	enum ad463x_lane_mode		lane_mode;
};

struct ad463x_state {
	struct spi_device		*spi;
	struct ad463x_phy_config	phy;

	union {
		unsigned char		buff[3];
		struct {
			unsigned short	reg_addr;
			unsigned char	reg_data;
		};
	} adc_data[2] ____cacheline_aligned;
};

static int ad463x_spi_read_reg(struct ad463x_state *st,
			       unsigned int reg_addr,
			       unsigned int *reg_data)
{
	struct spi_transfer xfer = {
		.tx_buf = st->adc_data[0].buff,
		.rx_buf = st->adc_data[1].buff,
		.len = 3,
		.bits_per_word = 8,
		.speed_hz = AD463X_SPI_REG_ACCESS_SPEED
	};
	int ret;

	st->adc_data[0].reg_addr = cpu_to_be16(AD463X_REG_READ_MASK(reg_addr));
	st->adc_data[0].reg_data = AD463X_REG_READ_DUMMY;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	*reg_data = (unsigned int)st->adc_data[1].reg_data;

	return ret;
}

static int ad463x_spi_write_reg(struct ad463x_state *st,
				unsigned int reg_addr,
				unsigned int reg_data)
{
	struct spi_transfer xfer = {
		.tx_buf = st->adc_data[0].buff,
		.len = 3,
		.bits_per_word = 8,
		.speed_hz = AD463X_SPI_REG_ACCESS_SPEED
	};

	st->adc_data[0].reg_addr = cpu_to_be16(AD463X_REG_WRITE_MASK(reg_addr));
	st->adc_data[0].reg_data = reg_data;

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad463x_spi_write_reg_masked(struct ad463x_state *st,
				       unsigned int reg_addr,
				       unsigned int mask,
				       unsigned int reg_data)
{
	unsigned int temp;
	int ret;

	ret = ad463x_spi_read_reg(st, reg_addr, &temp);
	if (ret < 0)
		return ret;

	return ad463x_spi_write_reg(st, reg_addr, ((temp & ~mask) | reg_data));
}

static int ad463x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	int ret;
	struct ad463x_state *st = iio_priv(indio_dev);

	if (readval) {
		ret = ad463x_spi_read_reg(st, reg, readval);
	} else {
		if (reg == AD463X_EXIT_CFG_MODE ||
		    reg == AD463X_REG_AVG ||
		    reg == AD463X_REG_MODES)
			return -EINVAL;
		ret = ad463x_spi_write_reg(st, reg, writeval);
	}

	return ret;
}

static int ad463x_set_reg_access(struct ad463x_state *st, bool state)
{
	unsigned int dummy;

	if (state)
		/* Send a sequence starting with "1 0 1" to the SPI bus*/
		return ad463x_spi_read_reg(st, AD463X_CONFIG_TIMING, &dummy);
	else
		return ad463x_spi_write_reg(st, AD463X_REG_EXIT_CFG_MODE,
					AD463X_EXIT_CFG_MODE);
}


static int ad463x_phy_init(struct ad463x_state *st)
{
	int ret;

	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_LANE_MODE_MSK,
					  st->phy.lane_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_CLK_MODE_MSK,
					  st->phy.clock_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_DATA_RATE_MODE_MSK,
					  st->phy.data_rate_mode);
	if (ret < 0)
		return ret;

	return ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					   AD463X_OUT_DATA_MODE_MSK,
					   st->phy.out_data_mode);
}

static int ad463x_get_string_index(const char *string,
				   const char * const *string_array,
				   size_t array_size,
				   unsigned int *idx)
{
	int i;

	for (i = 0; i < array_size; i++) {
		if (string_array[i]) {
			if (!strcmp(string, string_array[i])) {
				*idx = i;
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int ad463x_parse_dt(struct ad463x_state *st)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	struct fwnode_handle *fwnode;
	const char *data_rate_mode;
	const char *out_data_mode;
	const char *clock_mode;
	const char *lane_mode;
	int ret;

	fwnode = dev_fwnode(indio_dev->dev.parent);

	ret = fwnode_property_read_string(fwnode, "adi,lane-mode",
					  &lane_mode);
	if (ret < 0)
		return ret;
	ret = fwnode_property_read_string(fwnode, "adi,clock-mode",
					  &clock_mode);
	if (ret < 0)
		return ret;
	ret = fwnode_property_read_string(fwnode, "adi,data-rate-mode",
					  &data_rate_mode);
	if (ret < 0)
		return ret;
	ret = fwnode_property_read_string(fwnode, "adi,out-data-mode",
					  &out_data_mode);
	if (ret < 0)
		return ret;

	ret = ad463x_get_string_index(lane_mode, ad463x_lane_mdoes,
				      ARRAY_SIZE(ad463x_lane_mdoes),
				      &st->phy.lane_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_get_string_index(clock_mode, ad463x_clock_mdoes,
				      ARRAY_SIZE(ad463x_clock_mdoes),
				      &st->phy.clock_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_get_string_index(data_rate_mode, ad463x_data_rates,
				      ARRAY_SIZE(ad463x_data_rates),
				      &st->phy.data_rate_mode);
	if (ret < 0)
		return ret;

	return ad463x_get_string_index(out_data_mode, ad463x_out_data_modes,
				       ARRAY_SIZE(ad463x_out_data_modes),
				       &st->phy.out_data_mode);
}

static int ad463x_setup(struct ad463x_state *st)
{
	int ret;

	ret = ad463x_set_reg_access(st, true);
	if (ret < 0)
		return ret;

	return ad463x_phy_init(st);
}

static const struct spi_device_id ad463x_id_table[] = {
	{"ad4630-24",	ID_AD4630_24},
	{}
};
MODULE_DEVICE_TABLE(spi, ad463x_id_table);

static const struct of_device_id ad463x_of_match[] = {
	{ .compatible = "adi,ad4630-24" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad463x_of_match);

static const struct iio_info ad463x_infos[] = {
	[ID_AD4630_24] = {
		.debugfs_reg_access = &ad463x_reg_access,
	}
};

static int ad463x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad463x_state *st;
	unsigned long device_id;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;

	device_id = spi_get_device_id(st->spi)->driver_data;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad463x_infos[device_id];

	ret = ad463x_parse_dt(st);
	if (ret < 0) {
		dev_err(&spi->dev, "%s invalid devicetree configuration\n",
			indio_dev->name);
		return -EINVAL;
	}

	ret = ad463x_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "%s setup failed\n", indio_dev->name);
		return -ENOEXEC;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad463x_driver = {
	.driver = {
		.name = "ad463x",
		.of_match_table = ad463x_of_match,
	},
	.probe = ad463x_probe,
	.id_table = ad463x_id_table,
};
module_spi_driver(ad463x_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD463x ADC family driver");
MODULE_LICENSE("GPL v2");
