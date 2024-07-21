.. SPDX-License-Identifier: GPL-2.0

=======================
Kernel driver rpmsg_i2c
=======================

Author: Maxime Méré for STMicroelectronics <maxime.mere@st.com>

Description
-----------

rpmsg_i2c is a virtual I2C bus driver based on rpmsg framework.
It allows user to communicate with a remote processor like if it was connected
on an I2C bus.

This driver was initially designed to grant linux side to emulate I2C connection
with her internal coprocessor.

Message exchanged to emulate the I2C communication must respect the following
structure:

struct rpmsg_i2c_msg {
	u8 addr;
	u32 count;
	u8 result;
	u8 buf[0];
};

addr corresponds to the i2c slave address + the R/W bit, count is the number of
bytes contained inside the buffer, result is a flag used by the slave to indicate
an ACK or a NACK and buf is the data container.

This driver supports only the master mode, meaning that virtual devices defined in
the coprocessor are slave only. A communication between a slave and its master
works as follow:

In case of write:

	- the master sends the structure with the R/W bit low and the appropriate
	  "count" and "buffer" value and then wait for an ACK

	- the slave sends an empty rpmsg_i2c struct, except for "result" which will
	  be set to an ACK or a NACK

In case of read:

	- the master sends the structure with the R/W bit high and the others
	  fields empty and wait for the slave reply

	- the slave sends the structure back with the appropriate count and buffer
	  value and result is set to ACK
