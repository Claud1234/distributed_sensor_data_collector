#!/bin/bash

FILE=sql/database.sql
if test -f "$FILE"; then
    mv "$FILE" "${FILE}.bak"
fi

docker-compose run mysql bash -c "mysqldump -h mysql -P 3306 -u db_user -ptransport123 --no-data --no-tablespaces transport_ecosystem_management_db > /tmp/sql/database.sql"
