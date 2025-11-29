#
# This file is the powerctrl recipe.
#

SUMMARY = "Simple powerctrl application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://powerctrl.c \
	   file://Makefile \
	   file://powerctrl.service"

S = "${WORKDIR}"

inherit systemd

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "powerctrl.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 powerctrl ${D}${bindir}

	     install -d ${D}${systemd_unitdir}/system
         install -m 0644 ${WORKDIR}/powerctrl.service ${D}${systemd_unitdir}/system
}
FILES:${PN} += "${bindir}/powerctrl ${systemd_unitdir}/system/powerctrl.service"
