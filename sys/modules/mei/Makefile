.PATH:	${SRCTOP}/sys/dev/mei

KMOD=	mei
SRCS=	mei.c
SRCS+=	bus_if.h device_if.h pci_if.h

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
