dvhop
=====

DV-hop positioning algorithm for NS3.20

###Installing
Just download ns-allinone-3.20 and then:

Code must be unzipped and placed in ```/path/to/ns3/src/dvhop``` directory
The ```test_generator.py``` file may be used for generating layout files that can be read by the main dvhop program (requires python). Run ```python3 test_generator.py -h``` for usage details
Run ```./waf run "dvhop-example <args>"``` to run the dvhop program
Possible args:
```
--pcap, bool, Enable or disable writing PCAP traces
--printRoutes, bool, Enable or disable printing routing table dumps
--size, uint32, The total number of beacons and nodes to generate
--time, double, The amount of time the simulation should run for
--step, double, The distance between created nodes
--seed, uint32, The randomization seed
--beaconCount, uint32, The number of generated nodes that should be beacons
--loadFrom, string, File to load layout from (this will override size, step, and beaconCount args)
--criticalConditions, bool, Enable of disable running the simulation under critical conditions
```

