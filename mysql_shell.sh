#!/bin/bash
docker-compose run mysql mysql -h host.docker.internal -P 3306 -u db_user -ptransport123 transport_ecosystem_management_db
