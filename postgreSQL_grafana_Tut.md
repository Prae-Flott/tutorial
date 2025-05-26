# Database Tut (PostgreSQL+Grafana)

## PostgreSQL Installation

Follow the steps in:
[PostgreSQL Document](https://www.postgresql.org/download/linux/ubuntu/)

### Configure PostgreSQL to Listen on External Interfaces

Check the PostgreSQL version with:

```
psql --version
```

Open your `postgresql.conf`, with 

```
nano  /etc/postgresql/<version>/main/postgresql.conf
```

and find:

```
# - Connection Settings -

listen_addresses = '*'                  # what IP address(es) to listen on;
```

So that the server would accept the information from any other addresses.

And also check, if `port=5432`


### Grant Client Authentication

open `pg_hba.conf`, in Ubuntu it should be at `/etc/postgresql/<version>/main/pg_hba.conf`

Add a line permitting your client’s IP (or subnet). For example, to allow any host on example IP address ( eg: 192.168.1.0/24) using password auth:

```
# TYPE  DATABASE    USER      ADDRESS         METHOD
host    all         all       192.168.1.0/24  md5
```

change the user authentification for the Linux system, find the following line, change the `peer` to `md5`:

```
local   all             all                                     peer
```

Save the file and reload the config (no full restart needed):

```
sudo systemctl reload postgresql
```

## PostgreSQL Tutorial

## Grafana Installation

Follow the steps in:
[Grafana Document](https://grafana.com/docs/grafana/latest/setup-grafana/installation/debian/)

After installing Grafana, remember to open port 3000, so that you can access the Grafana server remotely:
```
sudo ufw allow 3000/tcp
```

Then open the Broser in another PC, run `http://<server-ip>:3000`, on same PC, it should be `http://localhost:3000`

## Grafana Tutorial
