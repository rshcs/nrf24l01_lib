### Improved nrf24l01 library

*This library covers almost all the main features of nrf24l01 and also tested extensively in industrial environment*

**Working principle (acknowledge payload):**

* Each node is assigned to a unique address
* Only enabled and used one data pipe address (instead of using all six of em) in each node
* If a one node wants to read data from another node, it has to send a request to the second node address.
* If the second node in reach, then send back the requested data as an ack_payload data packet.*

**Some of the nrf24l01 clone units (cheap ones you probably buy online)  do not support ack payload function and IRQ function**

