#!/bin/bash
docker exec database-mysql-1 bash -c "mysql -h host.docker.internal -P 3307 -u db_user -ptransport123 transport_ecosystem_management_db" < /home/pinch/workspace/finest_mobility/database/sql/database.sql
