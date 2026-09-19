#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>

const std::string path_to_header = "./src/";
const std::string header_name = "git_info.h";
const std::string temp_txt_name = "git_info.txt";

const std::string branch_define_name = "GIT_BRANCH";
const std::string commit_define_name = "GIT_COMMIT_HASH";
const std::string commit_msg_define_name = "GIT_COMMIT_MSG";

const std::string header_guard = "GIT_INFO_H";

int main(int argc, char** argv) {
	// Get the current branch name and output it to a temp text file
	system("git branch --show-current --no-color > git_info.txt");
	// Get the current commit and commit message and output it to the temp file
	system("git log -n1 --pretty=oneline >> git_info.txt");

	// Open the temp file
	std::ifstream git_info_txt(temp_txt_name);
	if (!git_info_txt.is_open()) {
		std::cout << "Failed to open '" << temp_txt_name << "'" << std::endl;
		return -1;
	}

	// Vars to be filled
	std::string branch;
	std::string commit_hash;
	std::string commit_msg;
	std::string line;

	// Get the branch
	std::getline(git_info_txt, line);
	branch = line;

	// Get the commit hash
	std::getline(git_info_txt, line);
	std::stringstream sstream;
	sstream << line;

	// Extract the first "word" from this line
	std::getline(sstream, commit_hash, ' ');
	// Grab the rest for the commit message
	std::string temp;
	while (std::getline(sstream, temp, ' ')) {
		commit_msg += temp;
		commit_msg += ' ';
	}

	// Escape all quotes or backslashes in the commit message
	for (size_t i = 0; i < commit_msg.size(); i++) {
		if (commit_msg[i] == '"' || commit_msg[i] == '\\') {
			commit_msg.insert(i, 1, '\\');
			i++;
		}
	}

	// Now we construct the header file in memory
	std::ostringstream header_contents;

	// Add a message saying this file was auto-generated
	header_contents << "// THIS IS AN AUTO-GENERATED FILE, DO NOT EDIT" << "\n\n";

	// Add a header guard
	header_contents << "#ifndef " << header_guard << "\n";
	header_contents << "#define " << header_guard << "\n\n";

	// Add the branch name
	header_contents << "#define " << branch_define_name << " \"" << branch << "\"\n";
	// Add the commit hash
	header_contents << "#define " << commit_define_name << " \"" << commit_hash << "\"\n";
	// Add the commit message
	header_contents << "#define " << commit_msg_define_name << " \"" << commit_msg << "\"\n\n";

	// Finish the header guard
	header_contents << "#endif // " << header_guard;

	// Clean up and remove the temp text file
	git_info_txt.close();
	std::filesystem::remove(temp_txt_name);

	// Only rewrite the header if it would actually change. Rewriting it
	// unconditionally bumps its mtime every build, which forces make to
	// recompile every translation unit that includes it.
	const std::string header_path = path_to_header + header_name;
	std::ifstream existing_header(header_path);
	if (existing_header.is_open()) {
		std::stringstream existing_contents;
		existing_contents << existing_header.rdbuf();
		existing_header.close();

		if (existing_contents.str() == header_contents.str()) {
			return 0;
		}
	}

	std::ofstream git_info_header(header_path);
	if (!git_info_header.is_open()) {
		std::cout << "Failed to open '" << header_path << "'" << std::endl;
		return -1;
	}

	git_info_header << header_contents.str();
	git_info_header.close();

	return 0;
}
