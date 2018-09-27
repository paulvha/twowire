#
# Makefile:
#	twowire - I2C library for the Raspberry Pi
#
#################################################################################
# This file is part of twowire:
#
#
#    twowire is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    twowire is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################

VERSION=1
DESTDIR?=/usr
PREFIX?=/local

LDCONFIG?=ldconfig

STATIC=libtwowire.a
DYNAMIC=libtwowire.so.$(VERSION)

DEBUG	= -O2
DEPS = twowire.h bcm2835.h
CC	= gcc
INCLUDE	= -I.
DEFS	= -D_GNU_SOURCE
CFLAGS	= $(DEBUG) $(DEFS) -Wformat=2 -Wall -Wextra -Winline $(INCLUDE) -pipe -fPIC
LIBS    = -lbcm2835

###############################################################################

SRC	= twowire.cpp
HEADERS = twowire.h
OBJ	= twowire.o

all:		$(DYNAMIC)

static:		$(STATIC)

$(STATIC):	$(OBJ)
	echo "[Link (Static)]"
	ar rcs $(STATIC) $(OBJ)
	ranlib $(STATIC)
#	@size   $(STATIC)

$(DYNAMIC):	$(OBJ)
	echo "[Link (Dynamic)]"
	$(CC) -shared -Wl,-soname,libtwowire.so$(WIRINGPI_SONAME_SUFFIX) -o libtwowire.so.$(VERSION) $(LIBS) $(OBJ)

.cpp.o:
	echo [Compile] $<
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY:	clean
clean:
	 echo "[Clean]"
	 rm -f $(OBJ) 

.PHONY:	tags
tags:	$(SRC)
	echo [ctags]
	ctags $(SRC)


.PHONY:	install
install:	$(DYNAMIC)
	echo "[Install Headers]"
	install -m 0755 -d	$(DESTDIR)$(PREFIX)/include
	install -m 0644 $(HEADERS)	$(DESTDIR)$(PREFIX)/include
	echo "[Install Dynamic Lib]"
	install -m 0755 -d	$(DESTDIR)$(PREFIX)/lib
	install -m 0755 libtwowire.so.$(VERSION)    $(DESTDIR)$(PREFIX)/lib/libtwowire.so.$(VERSION)
	ln -sf $(DESTDIR)$(PREFIX)/lib/libtwowire.so.$(VERSION)	$(DESTDIR)/lib/libtwowire.so
	$(LDCONFIG)

.PHONY:	install-static
install-static:	$(STATIC)
	echo "[Install Headers]"
	install -m 0755 -d	$(DESTDIR)$(PREFIX)/include
	install -m 0644 $(HEADERS)	$(DESTDIR)$(PREFIX)/include
	echo "[Install Static Lib]"
	install -m 0755 -d	$(DESTDIR)$(PREFIX)/lib
	install -m 0755 libtwowire.a $(DESTDIR)$(PREFIX)/lib

.PHONY:	uninstall
uninstall:
	echo "[UnInstall]"
	cd $(DESTDIR)$(PREFIX)/include/ && rm -f $(HEADERS)
	cd $(DESTDIR)$(PREFIX)/lib/     && rm -f libtwowire.*
	$(LDCONFIG)


.PHONY:	depend
depend:
	makedepend -Y $(SRC)

# DO NOT DELETE

twowire.o: twowire.h

