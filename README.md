# MyPCLTemplate
## 始め方
Mintでしかテストしてないけど...
Debian系linuxでdockerをインストールしてから

``` curl -s https://raw.githubusercontent.com/YoheiSaito/MyPCLTemplate/master/install.sh | bash ```

dockerでマウントしたいdatasetがある場合は 
~/.local/bin/pcl_builder内のdatasetを編集しましょう

## pcl_build
### init
``` pcl_build プロジェクト名 ```  

MyPCLTemplateがクローンされ, git initが実行される

### cmake
``` pcl_build cmake ```  

プロジェクトのcmakeが実行される

### make
``` pcl_build cmake ```  

プロジェクトのcmakeが実行される

### run
``` pcl_build {command} ```  

commandがdocker内のprojectファイルでcommandが実行される
commandを省略するとbashが実行される

## License
MIT
