# Multi-Function Thermometer (MFT) Firmware  
Firmware files for the three variants of the Cooper-Atkins Multi-Function Thermometer (MFT).  
* Version 2.x for McDonalds Japan  
* Version 3.x for the general market  
* Version 4.x for McDonalds Global  

## Table of Contents  
[Development Environment](#development-environment)  
[Project Structure](#project-structure)  
[How to Compile](#how-to-compile)  
[How to Program an MFT](#how-to-program-an-mft)  
[How to Release OTA Updates](#how-to-release-ota-updates)  

## Development Environment  
The MFT firmware is build with Silicon Labs' _Simplicity Studio_ using the _Gecko Bluetooth SDK suite_ and the _IAR C Compiler_  
* Simplicity Studio version SV4.1.14.1  
* IAR C Compiler version 7.80.1.11864  
* Gecko SDK Suite: Bluetooth 2.4.2.0, MCU 5.2.2.0 (v1.1.1)  

## Project Structure
The MFT firmware project consists of four separate _Simplicity Studio_ projects.  These are:  
1. The secure bootloader  
2. The McDonalds Global MFT application firmware  
This firmware starts with the major version 4 and the minor version is used to track revisions, eg., 4.00, 4.01, 4.02, ...  
3. The McDonalds Japan MFT application firmware  
This firmware starts with the major version 2 and the minor version is used to track revisions, eg., 2.10, 2.11, 2.12, ...  
4. The Standard MFT application firmware  
This firmware starts with the major version 3 and the minor version is used to track revisions, eg., 3.04, 3.05, 3.06, ...  

These four _Simplicity Studio_ projects are each [compiled](#how-to-compile) separately, then for each variant of the MFT, the appropriate application firmware is [combined](#combining-the-bootloader-and-application) with the bootloader using a batch script to create a single firmware image.  

The MFT firmware is also field-upgradable over Bluetooth using the Blue2 Reader app.  The firmware update files are Gecko Bootloader (*.gbl) files, sometimes referred to as "gobble" files.  The .gbl files are [created](#how-to-release-ota-updates) using a batch script, then uploaded to the firmware updates server.  

## How to Compile  
After cloning this repository, the four _Simplicity Studio_ project need to be compiled.  

Before continuing, ensure you have IAR C Compiler version 7.80.1.11864 installed and a valid license for it.  Additionally, ensure Gecko SDK Suite: Bluetooth 2.4.2.0, MCU 5.2.2.0 (v1.1.1) is installed in Simplicity Studio (version SV4.1.14.1).  

### Importing to _Simplicity Studio_  
For each project, in _Simplicity Studio_, click **File -> Import...** to open the **Import Project** window.  
In the **Select a project to import:** section, browse to the project folder.  
In the **Detected projects** section, select the project with the **Simplicity Studio (.sls)** type and click **Next**.  
In the **Build Configurations of the Project** window, accept the defaults and click **Next**.  
In the **Project Configuration** window, optionally rename the project, optionally select either the default location or a location of your choosing, and click **Finish**.  

### Generating Code
Once you have a project imported, open the project in the **Project Explorer** pane, then double-click the *.isc file to open it.  
.isc files should open in the App Builder view.  Click the **Generate** button to generate the app code based on the App Builder settings.  
Overwrite any previously generated code.  
After the code generation is completed, you can close the App Builder view.  

### Compiling 
After generating code, clean and build the project.  To do so, click **Project -> Clean...**, select your project if it is not already selected, and _deselect_ the **Start a build immediately** checkbox.  Then click **OK**.  
In the toolbar, click the down-arrow next to the hammer icon (Build).  Select the **IAR ARM - Default** option to begin the build process.  
Note: at this time, there will be three warnings from the InitDevice.c auto-generated file for the three application firmware projects.  The warning is: _Warning[Pe188]: enumerated type mixed with another type_ and is outside of our control, as it is part of an auto-generated code file.  

## How to Program an MFT  
To program an MFT with your compiled project code, you will need to first combine the bootloader and the application into a single image.  Programming an MFT also flashes encryption and signing keys.  These keys are not included in this repository and must be fetched separately before programming.  

### Fetching Keys  
The signing and encryption keys are securely stored in XXX. **_TODO: Describe the process of fetching the required keys for the MFT_**.  Unzip the entire **keys.zip** folder to the root of this repository, so a **keys** folder resides alongside the folders for the bootloader project and application firmware projects.  
```
├───.git
│   └───[...]
├───keys
├───MFT-bootloader
│   └───[...]
├───MFT-MCD-Global
│   └───[...]
├───MFT-MCD-Japan
│   └───[...]
├───MFT-Standard
│   └───[...]
└───releases
```

### Combining the Bootloader and Application  
Each application project has a subdirectory named **app-plus-bootloader**.  Inside this subdirectory are two batch files.  
* CreateCombined_MFT-{variant}.bat combines the application and bootloader into a single .s37 image.  
* Program_MFT-{variant}.bat will program an MFT connected to a J-Link.  

To combine the bootloader and the application, double-click the CreateCombined_MFT-{variant}.bat file to launch it.  It should end with a `!!!SUCCESS!!!` message.  If you do not see this message, read the error messages and troubleshoot the issue.  

### Programming an MFT board  
#### Equipment needed  
The MFT is programmed using a J-Link or Silicon Labs development kit with a debug adapter interface.  The solder side of the MFT circuit board (11-001323) has a standard 10-pin ARM Cortex Debug Header that connects to the BGM12x's JTAG/SWD interface.    

#### Programming script
A programming script is used to ensure the encryption and signing keys are flashed to the MFT and that the debug interface is locked out after programming.  
To program the MFT, connect the J-Link or SiLabs dev kit to the 10-pin header on the circuit board.  Double-click the Program_MFT-[variant].bat file to launch it.  It should end with a `!!!SUCCESS!!!` message.  If you do not see this message, read the error messages and troubleshoot the issue.  

## How to Release OTA Updates
Over-the-air updates are performed on the MFT with the Blue2 Reader app.  The Blue2 Reader app looks for available updates and downloads images from the Cooper-Atkins updates server located at _https://cooper-atkinsupdates.com_.  This is a machine-to-machine server and does not provide a user interface for browsing or downloading files.  When a new firmware version is available, it will need to be signed, encrypted, and uploaded to this server.  

### Signing and Encrypting a Firmware Image  
Each application firmware folder contains a batch script called **create_firmware_update_files.bat**.  Running this batch script will sign and encrypt the .out file located in the build output directory.  The resultant files will be located in the application firmware folder under the subdirectory output_gbl, and the one used for OTA updates is called _app-signed-encrypted.gbl_.  

### Preparing Update Files for Upload  
Located in the root directory there is a _releases_ directory with an _update-files_ subdirectory, as seen here:  
```
├───.git
├───keys
├───MFT-bootloader
├───MFT-MCD-Global
├───MFT-MCD-Japan
├───MFT-Standard
└───releases
    ├───[...]
    └───update-files
```
Inside the _update-files_ directory is a file called _firmwarefiles.xml_ with meta data that the Blue2 Reader app reads to check for updates.  Find the appropriate hardware version node for the MFT variant you are updating.  
| Variant | Firmware Version | Hardware Version | Notes |  
|:--- |:---:|:---:|:--- |  
| McDonalds Japan | 2.xx | 008 | |  
| McDonalds Global | 4.xx | 007 | This was the original, now obsolete 1.x firmware which now gets updated to 4.x |  
| McDonalds Global | 4.xx | 010 | This is the main McDonalds Global branch |  
| General Market | 3.xx | 009 | |  

Update the "version" attribute of the "firmware" tag to the new firmware version.  
Update the "filename" tag content to the name of the new .gbl file.  Note, the filename should include the version number to keep things straight on the server.  
Copy the new .gbl file to the _update-files_ folder.  

### Uploading to the Updates Server  
The firmware files and meta data are uploaded to the updates server using SFTP.  **_TODO: Describe how to get the SFTP server name and credentials_**.  Log into the SFTP server and upload the new firmware file and newly edited _firmwarefiles.xml_ file to the **blue2/** directory.  

Once the files are updated on the server, test that an MFT gets the correct firmware when updating with the Blue2 Reader app.   

