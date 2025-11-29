SUMMARY = "Simple tcp application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://tcp.c \
    file://protocol.h \
    file://Makefile \
    file://tcp.service \
    file://protocol.h"


S = "${WORKDIR}"

inherit systemd

SYSTEMD_SERVICE:${PN} = "tcp.service"

do_compile() {
    oe_runmake
}

do_install() {
    # 바이너리 설치
    install -d ${D}${bindir}
    install -m 0755 tcp ${D}${bindir}

    # 서비스 파일 설치
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/tcp.service ${D}${systemd_system_unitdir}
}

FILES:${PN} += "${systemd_system_unitdir}/tcp.service"
RDEPENDS:${PN} += "systemd"
