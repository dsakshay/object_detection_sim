FROM python:3.6

RUN pip install google-api-python-client requests

COPY ./application_default_credentials.json /
COPY ./get_event_details.py /

CMD ["python", "get_event_details.py"]
