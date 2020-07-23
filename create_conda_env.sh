# A shell script to setup the conda env of pybullet_fingers and install it within this env.

echo ~~~ Creating the conda env with the necessary and sufficient requirements. ~~~
conda env create -f environment.yml

echo ~~~ Initializing conda to run with the current shell ~~~
conda init $(basename ${SHELL})

echo ~~~ Activating the created conda env ~~~
conda activate pybullet_fingers

echo ~~~ Installing pybullet_fingers ~~~
python -m pip install -e .
