*****************
About Singularity
*****************


We are using Singularity_ to ensure that the code always runs in the same
environment (i.e. same versions of libraries installed, etc.), no matter if you
are running it locally or on our robots.

If you want to execute your code locally in simulation, all you need to
install is a recent version of Singularity and to download our image,
which contains all the necessary libraries (see the documentation of the specify
phase of the challenge for the download link).


.. _singularity_install:

Install Singularity
===================

We are using Singularity version 3.6.  Other recent versions are probably also
so fine, however, we cannot guarantee compatibility for those.
Unfortunately, most versions of Ubuntu still provide Singularity version 2.x in
their official repositories.  A newer version can be installed from source in
this case.  For this you may follow the `official installation instructions`_ or
use the following, slightly simplified instructions (assuming you are working
with Ubuntu).

Install system dependencies::

    $ sudo apt-get update && sudo apt-get install -y \
        build-essential \
        libssl-dev \
        uuid-dev \
        libgpgme11-dev \
        squashfs-tools \
        libseccomp-dev \
        wget \
        pkg-config \
        git \
        cryptsetup


Get the required version of the Go compiler::

    cd ~/Downloads  # you can save it anywhere else, just adjust paths below
    wget https://dl.google.com/go/go1.13.linux-amd64.tar.gz
    tar -xzf go1.13.linux-amd64.tar.gz

Note that it is only needed once for building singularity, so no need to install
it permanently (we just add it to PATH temporarily for building, see below).

Now download and unpack the singularity source::

    wget https://github.com/sylabs/singularity/releases/download/v3.6.1/singularity-3.6.1.tar.gz
    tar -xzf singularity-3.6.1.tar.gz


And finally build and install it::

    export PATH=~/Downloads/go/bin:${PATH}  # adjust path if you used a different directory
    cd singularity  # the folder to which the singularity source was extracted
    ./mconfig
    cd builddir
    make
    sudo make install


Now you should be able to use Singularity.  You can test this, for example, by
running ``singularity --version`` which should print "singularity version
3.6.1".  For more information on how to use Singularity, see the `official
documentation`_.


About the Base Image
==========================

We provide a Singularity image with Ubuntu 18.04 with all dependencies of the
the simulation and the packages needed to execute your code on the real robot already installed.
 For most of the libraries we use the default version provided through the official Ubuntu repositories.
To explore the image and to check which libraries and which versions exactly are
installed, you can run

::

    $ singularity shell path/to/blmc_ei_base.sif

To open a shell inside the image.



.. _singularity_extend_container:

Add Custom Dependencies to the Container
========================================

The image we provide already includes everything needed to run the robot and the
simulation.  However, you may need additional libraries to use them in our own
code, which are not yet present.  In this case, you can create your own image
which is based on our standard image but extends it with your additional
dependencies.


Create the User Image
---------------------

To extend the image, create *definition file* like the following:

::

    # Specify the name of the base image below
    Bootstrap: localimage
    From: ./blmc_ei_base.sif

    %post
        # Put commands to install additional dependencies here.
        # Make sure everything runs automatically without human input (e.g. add
        # `-y` to automatically say "yes" below).
        apt-get install -y package_name

See the official `Documentation for Definition Files`_ for all options in the
definition file.

Assuming you called your definition file ``user_image.def``, use the following
command to build the image.  Note that the base image (specified in the
``From:`` line) needs to be present in the directory in which you call the
command.

::

    $ singularity build --fakeroot user_image.sif path/to/user_image.def


.. warning::

   To ensure that your custom image is compatible with our setup for executing
   the code, always base it off blmc_ei_base and do not modify the runscripts
   or "apps" it defines.  Likewise, do not overwrite existing libraries or
   applications of the base image with different versions, as we cannot guarantee
   that our software will function correctly in this case.


.. _Singularity: https://sylabs.io/guides/3.6/user-guide/
.. _official documentation: https://sylabs.io/guides/3.6/user-guide/index.html
.. _Documentation for Definition Files: https://sylabs.io/guides/3.6/user-guide/definition_files.html
.. _official installation instructions: https://sylabs.io/guides/3.6/user-guide/quick_start.html#quick-installation-steps
