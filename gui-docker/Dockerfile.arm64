FROM nginx:alpine

LABEL authors="Gabriele Baldoni"

RUN apk update -qq && apk add git
RUN cd /root/ && git clone https://github.com/atolab/teleop_yaks
RUN cp -r /root/teleop_yaks/gui/* /usr/share/nginx/html/
RUN rm -rf /root/teleop_yaks
