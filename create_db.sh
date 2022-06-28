#!/bin/bash
docker-compose run mysql mysql -h mysql -P 3306 -u db_user -ptransport123 -e "create database transport_ecosystem_management_db"
