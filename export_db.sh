#!/bin/bash

FILE=sql/tem_db.sql
if test -f "$FILE"; then
    mv "$FILE" "${FILE}.bak"
fi

docker-compose run postgres pg_dump -h localhost -U db_user -d transport_ecosystem_management_db -f /tmp/sql/tem_db.sql
