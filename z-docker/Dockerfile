FROM ubuntu:bionic

LABEL authors="Gabriele Baldoni"

RUN apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install libev4 libssl1.1 curl
COPY zenohd /root/zenohd
COPY yaks-plugin.cmxs /root/yaks-plugin.cmxs

#RUN cd /root/ && curl ... && dpkg -i zenoh.deb
#CMD /etc/zenoh/zenohd -P "/etc/zenoh/yaks-plugin.cmxs -s /**" -vv

CMD /root/zenohd -P "/root/yaks-plugin.cmxs -s /**" -vv