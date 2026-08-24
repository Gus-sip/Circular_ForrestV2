# Module A

No local firmware. Module A is the central server the system reports up to -
a self-hosted ThingsBoard instance that Module B (`../module_b/communications/`)
uplinks to over NB-IoT/MQTT. This folder exists to keep the module_a/module_b/
module_c naming scheme complete, not because there's code to split here.

See `../notes/module_b_README.md` and `../module_b/communications/main.cpp`
for how Module B talks to it.
