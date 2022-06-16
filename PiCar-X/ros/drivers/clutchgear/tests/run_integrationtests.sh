#!/bin/sh
set -e
docker-compose -f docker-compose.yml up -d
sleep 15
docker exec -it tests-steering-dtp-1 /bin/bash -c "source devel/picarx_clutchgear_driver/setup.bash && rostest picarx_clutchgear_driver integration_tests.test"

docker-compose -f docker-compose.yml down

exec "$@"
