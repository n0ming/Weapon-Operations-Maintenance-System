#
# This file is the can recipe.
#

SUMMARY = "Simple can application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://can.c \
	   file://Makefile \
	   file://can.service \
	   file://protocol.h"

S = "${WORKDIR}"
inherit systemd 

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "can.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 can ${D}${bindir}

		install -d ${D}${systemd_unitdir}/system
        install -m 0644 ${WORKDIR}/can.service ${D}${systemd_unitdir}/system
}

FILES:${PN} += "${bindir}/can ${systemd_unitdir}/system/can.service" 
