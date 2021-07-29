sudo apt update && sudo apt upgrade -y
sudo docker pull yohei31to/pcl_build:latest
wget https://raw.githubusercontent.com/YoheiSaito/MyPCLTemplate/master/pcl_builder -O pcl_builder
chmod +x pcl_builder
install_path=$HOME/.local/bin/
if [[ -d $install_path ]]; then
  mv pcl_builder $install_path
else
  mkdir -p $install_path
  mv pcl_builder $install_path
  export PATH=$PATH:$(echo $install_path)
  echo "export PATH=\$PATH:$(echo $install_path)" >> $HOME/.profile
fi

xhost +local:user
