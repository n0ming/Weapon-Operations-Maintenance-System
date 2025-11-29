#
# This file is the errorcheck recipe.
#

SUMMARY = "Simple errorcheck application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://errorcheck.c \
    file://errorcheck.service \
    file://errorcheck.conf \
    file://app_ver.bin \
    file://fpga_ver.bin \
    file://Makefile \
    file://protocol.h"

S = "${WORKDIR}"

inherit systemd

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE:${PN} = "errorcheck.service"
SYSTEMD_AUTO_ENABLE = "enable"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} -O2 -Wall -o errorcheck errorcheck.c -lm
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${S}/errorcheck ${D}${bindir}/errorcheck

    install -d ${D}${sysconfdir}
    install -m 0644 ${S}/errorcheck.conf ${D}${sysconfdir}/errorcheck.conf

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${S}/errorcheck.service ${D}${systemd_system_unitdir}/errorcheck.service

    install -m 0644 ${WORKDIR}/app_ver.bin ${D}${sysconfdir}/app_ver.bin
    install -m 0644 ${WORKDIR}/fpga_ver.bin ${D}${sysconfdir}/fpga_ver.bin
}

FILES:${PN} += " \
    ${bindir}/errorcheck \
    ${sysconfdir}/errorcheck.conf \
    ${systemd_system_unitdir}/errorcheck.service \
    ${sysconfdir}/app_ver.bin \
    ${sysconfdir}/fpga_ver.bin \
"

RDEPENDS:${PN} += "systemd"
