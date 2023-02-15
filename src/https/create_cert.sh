#!/bin/bash

# Copyright (C) 2021 WZePaperDisplay Contributors
# Contact: SAI-Lab Berlin (https://www.chemie.tu-berlin.de/sai_lab/sei_real_labor_sai_lab)
# 
# This file is part of the WZePaperDisplay firmware.
#
# The WZePaperDisplay firmware is free software: you can
# redistribute it and/or modify it under the terms of the GNU
# Lesser General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option)
# any later version.

# WZePaperDisplay firmware is distributed in the hope that
# it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
# PURPOSE. See the GNU Lesser General Public License for more
# details.

# You should have received a copy of the GNU Lesser General Public
# License along with the WZePaperDisplay firmware. If not,
# see <http://www.gnu.org/licenses/>.

#  Based on https://github.com/openbikesensor/OpenBikeSensorFirmware/
#
#  The OpenBikeSensor firmware is free software: you can redistribute it
#  and/or modify it under the terms of the GNU Lesser General Public License as
#  published by the Free Software Foundation, either version 3 of the License,
#  or (at your option) any later version.
#
#  OpenBikeSensor firmware is distributed in the hope that it will be
#  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
#  General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with the OpenBikeSensor firmware.  If not, see
#  <http://www.gnu.org/licenses/>.
#

# based on https://raw.githubusercontent.com/fhessel/esp32_https_server/master/extras/create_cert.sh
# Certs must be updated from time to time,
# might be once we get a proper update mechanism
# in place. For now it is a static - all the same
# cert.
# Its sole purpose is to allow encrypted communication between the OBS
# and a local browser.

set -e
#------------------------------------------------------------------------------
# cleanup any previously created files
rm -f cbmca.* cbm.* cert.h private_key.h

#------------------------------------------------------------------------------
# create a CA called "myca"

# create a private key
openssl genrsa -out cbmca.key 1024

# create certificate
cat > cbmca.conf << EOF
[ req ]
distinguished_name     = req_distinguished_name
prompt                 = no
[ req_distinguished_name ]
C = DE
L = Berlin
O = adfc-tk.de 
CN = cbm-ca.local
EOF
openssl req -new -x509 -days 3650 -key cbmca.key -out cbmca.crt -config cbmca.conf
# create serial number file
echo "01" > cbmca.srl

#------------------------------------------------------------------------------
# create a certificate for the ESP (hostname: "WZePaperDisplay")

# create a private key
openssl genrsa -out cbm.key 1024
# create certificate signing request
cat > cbm.conf << EOF
[ req ]
distinguished_name     = req_distinguished_name
prompt                 = no
[ req_distinguished_name ]
C = DE
L = Berlin
O = adfc-tk.de 
CN = cbm.local
EOF
openssl req -new -key cbm.key -out cbm.csr -config cbm.conf

# have myca sign the certificate
openssl x509 -days 3650 -CA cbmca.crt -CAkey cbmca.key -in cbm.csr -req -out cbm.crt

# verify
openssl verify -CAfile cbmca.crt cbm.crt

# convert private key and certificate into DER format
openssl rsa -in cbm.key -outform DER -out cbm.key.DER
openssl x509 -in cbm.crt -outform DER -out cbm.crt.DER

# create header files
echo "#ifndef CERT_H_" > ./cert.h
echo "#define CERT_H_" >> ./cert.h
xxd -i cbm.crt.DER >> ./cert.h
echo "#endif" >> ./cert.h

echo "#ifndef PRIVATE_KEY_H_" > ./private_key.h
echo "#define PRIVATE_KEY_H_" >> ./private_key.h
xxd -i cbm.key.DER >> ./private_key.h
echo "#endif" >> ./private_key.h

echo ""
echo "Certificates created!"
echo "---------------------"
echo ""
echo "  Private key:      private_key.h"
echo "  Certificate data: cert.h"
echo ""
echo "Make sure to have both files available for inclusion when running the examples."
echo "The files have been copied to all example directories, so if you open an example"
echo " sketch, you should be fine."
