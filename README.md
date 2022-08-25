# mila4-unitree-go1-deployment

## Installation:
1. submodule init: `$ git submodule init .` and `$ git submodule update .`
2. install wrapper:
	```bash
	$ cd submodules/mila4-unitree-api-wrapper/lib
	
	# install robot interface lib [arm64] / x86 check original repo
	$ wget https://github.com/unitreerobotics/unitree_legged_sdk/raw/master/lib/python/arm64/robot_interface.cpython-38-aarch64-linux-gnu.so 
	
	# install editable wrapper lib
	$ cd ..
	$ pip install -e .	
	```
3. install lib to your local src:
	```bash
	$ cd src/
	$ wget https://github.com/unitreerobotics/unitree_legged_sdk/raw/master/lib/python/arm64/robot_interface.cpython-38-aarch64-linux-gnu.so
	```

## Deploy:
1. save your jit compiled `.pt` model into `submodules/mila4-unitree-api-wrapper/policies`
