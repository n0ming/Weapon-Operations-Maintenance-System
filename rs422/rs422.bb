#
# This file is the rs422 recipe.
#

SUMMARY = "Simple rs422 application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://rs422.c \
	   file://Makefile \
	   file://rs422.service \
	   file://protocol.h"

S = "${WORKDIR}"

inherit systemd

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "rs422.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 rs422 ${D}${bindir}/rs422

	     install -d ${D}${systemd_unitdir}/system
             install -m 0644 ${WORKDIR}/rs422.service ${D}${systemd_unitdir}/system
}
FILES:${PN} += "${bindir}/rs422 ${systemd_unitdir}/system/rs422.service"
