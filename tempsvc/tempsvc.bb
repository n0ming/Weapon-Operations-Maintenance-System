#
# This file is the tempsvc recipe.
#

SUMMARY = "Simple tempsvc application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
SRC_URI = "file://tempsvc.c \
           file://Makefile \
           file://tempsvc.service \
           file://protocol.h"


S = "${WORKDIR}"

inherit systemd

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "tempsvc.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 tempsvc ${D}${bindir}

             install -d ${D}${systemd_unitdir}/system
             install -m 0644 ${WORKDIR}/tempsvc.service ${D}${systemd_unitdir}/system

}

FILES:${PN} += "${bindir}/temp ${systemd_unitdir}/system/tempsvc.service"

