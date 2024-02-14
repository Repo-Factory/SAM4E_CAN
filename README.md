# SAM4E_CAN

This Project aims to create a CAN Driver targeted for the SAM4E platform using an XplainedPro board.

## Building/Linking/Flashing

Atmel encourages their use of Atmel Studio to deal with the workflow of the SAM chip, but it is only available on Windows. Instead here, a custom CMakeLists.txt was made for the build and openocd is utilized for the flashing process. Please refer to the [compile.txt](compile.txt) file to see a bit on the building and flashing process for the chip from linux.

## Getting Started

Clone this repository

    git clone https://github.com/Repo-Factory/SAM4E_CAN

`ASF` - Will hold some libraries and modules included from manufacturer with header definitions and functions to use
`projects` - Will hold source code individual to this project 