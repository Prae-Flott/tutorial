# Database Tut (PostgreSQL+Grafana)

## PostgreSQL Installation

Follow the steps in:
[PostgreSQL Document](https://www.postgresql.org/download/linux/ubuntu/)

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
