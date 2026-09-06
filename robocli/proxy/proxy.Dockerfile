# The proxy: the only path from the sandbox to the outside world.
# tinyproxy, default-deny; the whitelist is a generic build arg (one
# regex per line). Empty (the default) = deny everything. Callers whose
# agent needs an API pass its host regexes as the value.
FROM alpine:3.20
RUN apk add --no-cache tinyproxy
COPY tinyproxy.conf /etc/tinyproxy/tinyproxy.conf
ARG WHITELIST=""
RUN printf '%s\n' "${WHITELIST}" > /etc/tinyproxy/filter
# The listen port is a build parameter; the image DESCRIBES its own port
# via a label so `up` composes the URL from the artifact instead of a
# copied constant (single source: this ARG).
ARG PORT=8888
RUN sed -i "s/^Port .*/Port ${PORT}/" /etc/tinyproxy/tinyproxy.conf
LABEL robocli.proxy.port=${PORT}
CMD ["tinyproxy", "-d"]
