# wrench_cone

Build the H- and V-Rep of Wrench cone for arbitary contacts

### Installation

    wget https://github.com/cddlib/cddlib/releases/download/0.94m/cddlib-0.94m.tar.gz
    tar zxf cddlib-*.tar.gz
    cd cddlib-*
    ./configure
    make
    sudo make install

### Installation on H1s fucked up system

    sudo apt-get install m4

    tar -xvf autoconf-2.69.tar.xz
    ./configure --prefix=/opt/tools
    sudo make && make install
    export PATH=$PATH:/opt/tools/bin

    tar -xvf automake-1.15.tar.xz
    ./configure --prefix=/opt/tools
    sudo make && make install

    tar zxf cddlib-*.tar.gz
    cd cddlib-*
    ./configure
    sudo make && make install

### Reading

    http://www.roboticsproceedings.org/rss11/p28.pdf

    https://hal.archives-ouvertes.fr/hal-02108449/document

    http://hirailab.com/pub-books/Hirai-PhD-thesis.pdf

### Performance

Could mabye handle 8-9 contacts within the 250 Hz skin update time

    n_contacts= 4 n_constraints=32 elapsed= 159 us
    n_contacts= 5 n_constraints=45 elapsed= 305 us
    n_contacts= 6 n_constraints=50 elapsed= 418 us
    n_contacts= 7 n_constraints=111 elapsed= 1057 us
    n_contacts= 8 n_constraints=140 elapsed= 1388 us
    n_contacts= 9 n_constraints=189 elapsed= 2151 us
    n_contacts= 10 n_constraints=186 elapsed= 2563 us
    n_contacts= 11 n_constraints=295 elapsed= 4328 us
    n_contacts= 12 n_constraints=360 elapsed= 6117 us
    n_contacts= 13 n_constraints=421 elapsed= 8291 us
    n_contacts= 14 n_constraints=406 elapsed= 8646 us
    n_contacts= 15 n_constraints=575 elapsed= 13544 us
    n_contacts= 16 n_constraints=660 elapsed= 16089 us
    n_contacts= 17 n_constraints=749 elapsed= 23355 us
    n_contacts= 18 n_constraints=730 elapsed= 24009 us
    n_contacts= 19 n_constraints=955 elapsed= 37663 us


