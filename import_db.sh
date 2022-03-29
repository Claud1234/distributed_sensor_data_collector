#!/bin/bash
docker-compose run postgres psql -h localhost -U db_user -d transport_ecosystem_management_db -f /tmp/sql/tem_db.sql
