# install dependencies (curl)
sudo apt install curl

# makes the keyrings directory with correct permission
sudo mkdir -p -m0755 /etc/apt/keyrings

# downloads the key for the repo for apt to install from
sudo curl https://download.koromix.dev/debian/koromix-archive-keyring.gpg -o /etc/apt/keyrings/koromix-archive-keyring.gpg

# adds the repo to the apt list
echo "deb [signed-by=/etc/apt/keyrings/koromix-archive-keyring.gpg] https://download.koromix.dev/debian stable main" | sudo tee /etc/apt/sources.list.d/koromix.dev-stable.list

# updates apt
sudo apt update

# install tytools
sudo apt install -y tytools
