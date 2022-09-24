# Task
FROM ubuntu:20.04

# Task
RUN apt update && apt install -y python3

# Task
COPY ./example_program.py /
COPY ./entrypoint.sh /  

ENTRYPOINT ["/entrypoint.sh"]