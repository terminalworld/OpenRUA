# The proxy: the only path from the sandbox to the outside world when a
# run limits the agent to its model service. tinyproxy, default-deny;
# the whitelist (one regex per line) arrives as the WHITELIST variable at
# container start and is written to the filter file there, so one image
# serves every whitelist. Empty = deny everything.
FROM alpine:3.20
RUN apk add --no-cache tinyproxy
COPY tinyproxy.conf /etc/tinyproxy/tinyproxy.conf
# The listen port is a build parameter; the image DESCRIBES its own port
# via a label so `up` composes the URL from the artifact instead of a
# copied constant (single source: this ARG).
ARG PORT=8888
RUN sed -i "s/^Port .*/Port ${PORT}/" /etc/tinyproxy/tinyproxy.conf
LABEL openrua.proxy.port=${PORT}
ENV WHITELIST=""
CMD ["sh", "-c", "printf '%s\\n' \"$WHITELIST\" > /etc/tinyproxy/filter && exec tinyproxy -d"]
