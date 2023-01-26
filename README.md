# 4bf_drone

<details>
  <summary>Setting up Docker</summary>

### Setting up Docker
* Instructions for installing docker.io are included in the previous setup step for Ubuntu
* We created many scripts to simplify our setup. First, ensure you are cd'd into our repo folder. Then cd into the docker folder: `cd docker`
* To add permissions for the docker, run the following commands:

   `sudo groupadd docker`

   `sudo usermod -aG docker $USER`

   `newgrp docker`

* To check that docker is installed and running correctly, run in (when cd'd into docker folder) `docker run hello-world`
* Build our docker by running the script (still inside the docker folder): `bash ./start_docker`
* Wait while all the dependencies are loaded. Once everything is loaded, you should see that your terminal command line changes from your local computer name and the "$" should change to "root@${CONTAINER_ID}:~/utfr_ws#". Double-check if this does not appear to ensure you're in the correct folder. 

* If we ever mention the need to rebuild docker, this can be done with `source ./docker/rebuild_docker`

</details>
