# Khepera III (K3) UDP Driver

A UDP server that can be used to control the Khepera III (K3) mobile robot over the network. Accepts left and right wheel speeds, as well as, a request for the IR and encoder data.
 
The non-threaded version handles control and data sequentially on the same port, while the threaded version can handle control and data simultaneously on the same port.

## Protocol

For each of the three requests, a reply is sent back.

### Initialization

$K3DRV,REQ,INIT
$K3DRV,RES,INIT

### Control

$K3DRV,REQ,CTRL,R,L
$K3DRV,RES,CTRL

R,L are the right and left raw wheel speeds.

### Data

$K3DRV,REQ,DATA
$K3DRV,RES,DATA,IRC,IR0,...,IR10,ENC,EN0,EN1

Currently, only IR and encoder data is sent back.

## Requirements

This code compiles against software provided by the Khepera III Toolbox. This toolbox is freely available [here](http://en.wikibooks.org/wiki/Khepera_III_Toolbox).

## Credits

Author: Jean-Pierre de la Croix
Last Modified: 21 November 2011

Based on code from the Khepera III Toolbox by:
(c) 2006-2008 EPFL, Lausanne, Switzerland
Thomas Lochmatter
