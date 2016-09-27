This repository contains my work regarding r8169 linux driver. Motivation was preparing minimal driver (in sense of source code) which:

* works with real PCIe device RTL8111/8168
* handle probe() procedure via insmod
* handle open() procedure
* handle close() procedure
* handle remove() procedure via rmmod
* perform full autonegotiation with link partner
* handle events related to link state (plugging cable trigger LOWER_UP + Link Up)
* setup interface to LOWER_UP state

Because final driver is quite big, fragile and ugly I decided to extract each iteration to separated files: 
r8169.0.c, r8169.1.c etc.
