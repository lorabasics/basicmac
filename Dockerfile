FROM alpine:edge

ENV PROJECT_PATH=/basicmac

RUN mkdir -p $PROJECT_PATH
WORKDIR $PROJECT_PATH

RUN echo "http://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories

RUN apk add --no-cache \
	ca-certificates \
	make \
	git \
	bash \
	gcc-arm-none-eabi \
	newlib-arm-none-eabi \
	python3 \
	openocd \
	alpine-sdk \
	screen

RUN pip3 install \
	Click \
	intelhex \
	PyYAML

ADD tools/openocd/stlink-rules.tgz /etc/udev/rules.d/

