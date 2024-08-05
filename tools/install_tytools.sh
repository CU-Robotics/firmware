# adds something (required idk)
mkdir -p -m0755 /etc/apt/keyrings

# downloads the repo for apt to install from
curl https://download.koromix.dev/debian/koromix-archive-keyring.gpg -o /etc/apt/keyrings/koromix-archive-keyring.gpg

# adds the repo for apt to install from
echo "deb [signed-by=/etc/apt/keyrings/koromix-archive-keyring.gpg] https://download.koromix.dev/debian stable main" > /etc/apt/sources.list.d/koromix.dev-stable.list

# updates apt
apt update

# install tytools
apt install -y tytools
