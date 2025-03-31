# NEBP-Ground-Station-Tracker
Code for BPP's ground station tracking system for high-altitude balloon flights. Originally created during the Montana Space Grant Consortium's BOREALIS program in the summer of 2021.

## Initial Set-Up and Usage
> [!CAUTION]
> These steps haven't actually been tested. Follow at your own risk

> [!WARNING]  
> The below steps are intended for use on Debian-derived Linux distributions, such as Ubuntu. They may or may not work on systems running other operating systems.

 1. Clone this repository
 2. Initialize submodules using `git submodule update --init`
    - The Direwolf submodule needs to be built by installing the needed build tools and running the following from this repo's directory:
        ```
        cd external/direwolf
        mkdir build && cd build
        cmake ..
        make -j4
        ```
 3. Create Python venv and install pip packages from [requirements.txt](./requirements.txt)
 4. Run `npm install` in the [embedded_map](./embedded_map/) directory to install Node packages for the embedded map
 
 The program can then be run by sourcing the Python venv's activate script and running `python NEBP-Ground-Station-Tracker/main.py`

## Current Features
Features that are currently implemented and functional to some degree
 - Tracks a balloon location based on points from one of:
   - MSGC's Borealis flight tracking service
   - APRS.fi's API
   - APRS-IS
   - APRS packets from an attached SDR or serial TNC
   - A local CSV file of previously logged points
 - Logs received points to a local CSV file
 - Ability to manually and automatically control an NEBP automatic antenna pointing ground station
 - Maps received coordinates and plots received altitude vs time
 - Calculates azimuth, elevation, and line of sight distance from the ground station location to the balloon

## Potential Features
Features that might get added at some point (no guarantees)
 - New position update sources:
   - Direct from Iridium (rather than through Borealis)
   - RockBLOCK
   - SPOT tracker (doesn't do altitude, does have an easy XML/JSON API feed)
     - Can't use for pointing without altitude, but could visualize and use to known when the balloon is landed
 - Updating from multiple sources simultaneously
   - Should just be spawning a new updates thread for each source, adding a flag for which source to track with, and updating the GUI
 - Tracking from multiple sources simultaneously
   - Will want some form of sensor/data fusion to get more accurate positions from using multiple data sources
 - Automatic landing prediction
   - Maybe also automatic driving directions to the predicted landing site
 - Automatically fine-tuning the antenna pointing using radio direction finding
 - A way to save the map of received coordinates (as an image or something)
 - A server mode for the mapping so that clients on the same network can view the dynamically-updating map of received coordinates from their browser
 - Ability to select a different CSV or a previous log file for the test source from the GUI (rather than hardcoded to use data/test.csv)

## Known Issues
 - Program code, especially main.py, is not well organized and lacks documentation
 - About page needs images
 - About page text does not resize or get a scroll wheel when program window shrinks
   - Other text in the program (buttons and labels) also does not resize
   - No minimum window size when using WSLg, window can be shrunk to practically nothing
 - Terminology used throughout the program needs to be reviewed and standardized
   - Some confusion in terms after separating position updating and actual ground station pointing functionalities
   - Maybe use pointing for what's current referred to as tracking and tracking for what's currently referred as updating
 - APRS functions are not well tested and likely do not work well yet
   - APRS-IS source does not properly return altitude data, seems to work well otherwise
 - Setting the ground station location after starting position updating does not properly remove the prior balloon ground track (should it remove the ground track at all?)
   - Inconsistently removes only the most recent position marker
 - Altitude graph resets when changing ground station location (should this be a bug or intended behavior?)
 - Altitude graph resizes as points are added and labels (particularly time labels) overlap
 - Qthreads in main.py don't seem to be emitting the finished signal, so program cleanup holds until it manually sends the quit signal
   - Exception gets thrown in cleanup if the updates thread has already been stopped by the "End Connection to Selected Source" button or the HTTP server was never started (never switched to Location/Orientation tab)
 - Occasional crash due to "OSError: [Errno 98] Address already in use" when starting HTTP server after program restart
   - Probable fix: Need to make sure port bindings are finished being undone at program exit, add in a catch when starting the HTTP server and expose a GUI button to re-attempt binding, and/or use a random open ephemeral port for the server
 - Map overlay layers don't update until after the flyTo animation from setting the ground station location finishes (leaflet bug?)
 - Repeatedly switching to the map tab, either from the altitude graph or from a different page, can cause the map view to grow vertically (QWebEngineView problem?)
 - sipPyTypeDict() deprecation warnings show in terminal on program start (QWebEngineView problem?)
 
