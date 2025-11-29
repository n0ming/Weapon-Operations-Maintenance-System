#
# This file is the mic recipe.
#

SUMMARY = "Simple mic application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://mic.c \
	   file://mic.service \
	   file://Makefile \
		  "

S = "${WORKDIR}"
inherit systemd

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "mic.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 mic ${D}${bindir}

	     install -d ${D}${systemd_unitdir}/system
         install -m 0644 ${WORKDIR}/mic.service ${D}${systemd_unitdir}/system
}

FILES:${PN} += "${bindir}/mic ${systemd_unitdir}/system/mic.service" 
