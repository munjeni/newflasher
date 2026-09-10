//######################################
//#######    Xperia Downloader   #######
//#######     Savan @ 2026     #######
//######################################
//
//
// gcc -Wall -O3 -march=native -pipe -I/usr/include/libxml2 sony.c -o sony -lcurl -lxml2
//
//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <signal.h>
#include <curl/curl.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <time.h>
#include <pthread.h>

// Global control for debug log outputs (set via main arguments)
static int debug_enabled = 0;

// Macro that prints logs only when debug is enabled
#define LOG(...) do { if (debug_enabled) { printf(__VA_ARGS__); } } while (0)

struct MemoryStruct
{
	char *memory;
	size_t size;
};

struct ProgressData
{
	int chunk_index;
	double last_runtime;
};

// Function prototypes
void parseWeatherXML(const char *xml_content);
void parseSoftwareDetailsXML(xmlNodePtr root_element);
void parseFileChunksXML(xmlNodePtr root_element, const char *target_filename, const char *total_expected_size_str);
long long download_chunk_to_file(const char *url, const char *filename, int append_mode, int chunk_index);
long long get_final_file_size(const char *filename);
int download_xml(const char *url, struct MemoryStruct *chunk);
size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);
size_t WriteFileCallback(void *ptr, size_t size, size_t nmemb, FILE *stream);
int progress_callback(void *clientp, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);

// Global static buffers with explicit array sizes
static char current_product[64] = {0};
static char current_model[64] = {0};
static char current_name[128] = {0};
static char current_release_state[64] = {0};
static char current_download_url[1024] = {0};
static char current_software_version[64] = {0};
static char current_android_version[16] = {0};

size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
	size_t realsize = size * nmemb;
	struct MemoryStruct *mem = (struct MemoryStruct *)userp;
	char *ptr = realloc(mem->memory, mem->size + realsize + 1);
	
	if (!ptr)
	{
		printf("Not enough memory!\n");
		return 0;
	}
	
	mem->memory = ptr;
	memcpy(&(mem->memory[mem->size]), contents, realsize);
	mem->size += realsize;
	mem->memory[mem->size] = 0;
	return realsize;
}

size_t WriteFileCallback(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
	return fwrite(ptr, size, nmemb, stream);
}

int progress_callback(void *clientp, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow)
{
	struct ProgressData *data = (struct ProgressData *)clientp;
	
	if (dltotal > 0)
	{
		double percentage = ((double)dlnow / (double)dltotal) * 100.0;
		int bar_width = 30;
		int progress = (int)((percentage / 100.0) * bar_width);

		printf("\r Chunk %d [", data->chunk_index);
		for (int i = 0; i < bar_width; ++i)
		{
			if (i < progress)
			{
				printf("=");
			}
			else
			{
				printf(" ");
			}
		}
		printf("] %.2f%% (%ld/%ld B)", percentage, (long)dlnow, (long)dltotal);
		fflush(stdout);
	}
	return 0;
}

long long download_chunk_to_file(const char *url, const char *filename, int append_mode, int chunk_index)
{
	CURL *curl_handle = curl_easy_init();
	CURLcode res;
	FILE *fp;
	struct ProgressData progress_data;

	if (curl_handle == NULL)
	{
		return -1;
	}

	if (append_mode)
	{
		fp = fopen(filename, "ab");
	}
	else
	{
		fp = fopen(filename, "wb");
	}

	if (fp == NULL)
	{
		curl_easy_cleanup(curl_handle);
		return -1;
	}

	progress_data.chunk_index = chunk_index;
	progress_data.last_runtime = 0;

	curl_easy_setopt(curl_handle, CURLOPT_URL, url);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteFileCallback);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, fp);
	curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
	curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 0L); 

	curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 0L);
	curl_easy_setopt(curl_handle, CURLOPT_XFERINFOFUNCTION, progress_callback);
	curl_easy_setopt(curl_handle, CURLOPT_XFERINFODATA, &progress_data);

	curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 0L);
	curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYHOST, 0L);

	res = curl_easy_perform(curl_handle);

	long long bytes_written = 0;
	if (res == CURLE_OK)
	{
		bytes_written = ftell(fp);
	}

	fclose(fp);
	curl_easy_cleanup(curl_handle);

	if (res != CURLE_OK)
	{
		return -1;
	}

	return bytes_written;
}

long long get_final_file_size(const char *filename)
{
	FILE *fp = fopen(filename, "rb");
	if (fp == NULL)
	{
		return -1;
	}
	fseek(fp, 0, SEEK_END);
	long long size = ftell(fp);
	fclose(fp);
	return size;
}

int download_xml(const char *url, struct MemoryStruct *chunk)
{
	CURL *curl_handle = curl_easy_init();
	CURLcode res;

	if (curl_handle == NULL)
	{
		return 0;
	}

	curl_easy_setopt(curl_handle, CURLOPT_URL, url);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)chunk);
	curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
	curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 5L);

	curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 0L);
	curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYHOST, 0L);

	res = curl_easy_perform(curl_handle);
	curl_easy_cleanup(curl_handle);

	if (res != CURLE_OK)
	{
		return 0;
	}

	return 1;
}

void parseFileChunksXML(xmlNodePtr root_element, const char *target_filename, const char *total_expected_size_str)
{
	long long total_expected_size = strtoll(total_expected_size_str, NULL, 10);
	long long current_file_offset = 0;

	printf("\n--- STARTING DOWNLOAD: %s ---\n", target_filename);
	LOG("Expected final size: %lld bytes\n", total_expected_size);

	for (xmlNodePtr node = root_element->children; node != NULL; node = node->next)
	{
		if (node->type == XML_ELEMENT_NODE && strcmp((const char *)node->name, "file-chunk") == 0)
		{
			xmlChar *number = xmlGetProp(node, (const xmlChar *)"number");
			char temp_size[32] = {0};
			char temp_url[2048] = {0};
			int chunk_num = 0;

			if (number)
			{
				chunk_num = atoi((const char*)number);
			}

			for (xmlNodePtr child = node->children; child != NULL; child = child->next)
			{
				if (child->type == XML_ELEMENT_NODE)
				{
					if (strcmp((const char *)child->name, "size") == 0)
					{
						xmlChar *content = xmlNodeGetContent(child);
						if (content)
						{
							snprintf(temp_size, sizeof(temp_size), "%s", (const char*)content);
							xmlFree(content);
						}
					}
					else if (strcmp((const char *)child->name, "link") == 0)
					{
						xmlChar *href = xmlGetProp(child, (const xmlChar *)"href");
						if (href)
						{
							snprintf(temp_url, sizeof(temp_url), "%s", (const char*)href);
							xmlFree(href);
						}
					}
				}
			}

			long long expected_chunk_size = strtoll(temp_size, NULL, 10);

			if (strlen(temp_url) > 0)
			{
				int append = (chunk_num == 0) ? 0 : 1;
				
				long long file_size_after_download = download_chunk_to_file(temp_url, target_filename, append, chunk_num);
				printf("\n");

				if (file_size_after_download < 0)
				{
					fprintf(stderr, "Critical error: Chunk %d download was interrupted!\n", chunk_num);
					if (number) xmlFree(number);
					return;
				}

				long long actual_chunk_bytes = file_size_after_download - current_file_offset;
				
				if (actual_chunk_bytes != expected_chunk_size)
				{
					fprintf(stderr, "Error: Chunk %d has %lld bytes, but XML expected %lld! Aborting.\n", chunk_num, actual_chunk_bytes, expected_chunk_size);
					if (number) xmlFree(number);
					return;
				}

				current_file_offset = file_size_after_download;
			}

			if (number)
			{
				xmlFree(number);
			}
		}
	}

	long long final_disk_size = get_final_file_size(target_filename);
	if (final_disk_size == total_expected_size)
	{
		printf("SUCCESS: File %s is complete and verified! (%lld/%lld bytes)\n", target_filename, final_disk_size, total_expected_size);
	}
	else
	{
		fprintf(stderr, "CRITICAL ERROR: Merged file size is %lld bytes, expected %lld!\n", final_disk_size, total_expected_size);
	}
	printf("-----------------------------------------\n");
}

void parseSoftwareDetailsXML(xmlNodePtr root_element)
{
	for (xmlNodePtr node = root_element->children; node != NULL; node = node->next)
	{
		if (node->type == XML_ELEMENT_NODE)
		{
			if (strcmp((const char *)node->name, "software-version") == 0)
			{
				xmlChar *content = xmlNodeGetContent(node);
				if (content)
				{
					snprintf(current_software_version, sizeof(current_software_version), "%s", (const char*)content);
					xmlFree(content);
				}
			}
			else if (strcmp((const char *)node->name, "android-version") == 0)
			{
				xmlChar *content = xmlNodeGetContent(node);
				if (content)
				{
					snprintf(current_android_version, sizeof(current_android_version), "%s", (const char*)content);
					xmlFree(content);
				}
			}
		}
	}

	LOG("\n--- SONY SOFTWARE DETAILS ---\n");
	LOG("Software Version: %s\n", current_software_version);
	LOG("Android Version: %s\n", current_android_version);
	LOG("-----------------------------\n");

	for (xmlNodePtr node = root_element->children; node != NULL; node = node->next)
	{
		if (node->type == XML_ELEMENT_NODE && strcmp((const char *)node->name, "file-resources") == 0)
		{
			int file_counter = 1;

			for (xmlNodePtr res_node = node->children; res_node != NULL; res_node = res_node->next)
			{
				if (res_node->type == XML_ELEMENT_NODE && strcmp((const char *)res_node->name, "file-resource") == 0)
				{
					char temp_key[64] = {0};
					char temp_name[256] = {0};
					char temp_size[32] = {0};
					char temp_url[1024] = {0};

					for (xmlNodePtr child = res_node->children; child != NULL; child = child->next)
					{
						if (child->type == XML_ELEMENT_NODE)
						{
							if (strcmp((const char *)child->name, "file-key") == 0)
							{
								xmlChar *content = xmlNodeGetContent(child);
								if (content)
								{
									snprintf(temp_key, sizeof(temp_key), "%s", (const char*)content);
									xmlFree(content);
								}
							}
							else if (strcmp((const char *)child->name, "name") == 0)
							{
								xmlChar *content = xmlNodeGetContent(child);
								if (content)
								{
									snprintf(temp_name, sizeof(temp_name), "%s", (const char*)content);
									xmlFree(content);
								}
							}
							else if (strcmp((const char *)child->name, "plain-size") == 0)
							{
								xmlChar *content = xmlNodeGetContent(child);
								if (content)
								{
									snprintf(temp_size, sizeof(temp_size), "%s", (const char*)content);
									xmlFree(content);
								}
							}
							else if (strcmp((const char *)child->name, "link") == 0)
							{
								xmlChar *href = xmlGetProp(child, (const xmlChar *)"href");
								if (href)
								{
									snprintf(temp_url, sizeof(temp_url), "%s", (const char*)href);
									xmlFree(href);
								}
							}
						}
					}

					LOG("[File %d]\n", file_counter);
					LOG("Type/Key:  %s\n", temp_key);
					LOG("Name:      %s\n", temp_name);
					LOG("Size:      %s bytes\n", temp_size);
					LOG("Link:      %s\n", temp_url);
					LOG("\nLoading chunks for this file...\n");

					if (strlen(temp_url) > 0)
					{
						struct MemoryStruct chunk_file;
						chunk_file.memory = malloc(1);
						chunk_file.size = 0;

						if (download_xml(temp_url, &chunk_file))
						{
							xmlDocPtr sub_doc = xmlParseMemory(chunk_file.memory, strlen(chunk_file.memory));
							if (sub_doc)
							{
								xmlNodePtr sub_root = xmlDocGetRootElement(sub_doc);
								if (sub_root && strcmp((const char *)sub_root->name, "file-chunks") == 0)
								{
									parseFileChunksXML(sub_root, temp_name, temp_size);
								}
								xmlFreeDoc(sub_doc);
							}
						}
						else
						{
							fprintf(stderr, "Error opening file link: %s\n", temp_key);
						}

						free(chunk_file.memory);
					}

					file_counter++;
				}
			}
		}
	}
	LOG("-----------------------------\n");
}

void parseWeatherXML(const char *xml_content)
{
	xmlDocPtr doc = xmlParseMemory(xml_content, strlen(xml_content));

	if (doc == NULL)
	{
		fprintf(stderr, "Error parsing XML.\n");
		return;
	}

	xmlNodePtr root_element = xmlDocGetRootElement(doc);
	if (root_element == NULL)
	{
		xmlFreeDoc(doc);
		return;
	}

	if (strcmp((const char *)root_element->name, "match-response") == 0)
	{
		for (xmlNodePtr node = root_element->children; node != NULL; node = node->next)
		{
			if (node->type == XML_ELEMENT_NODE && strcmp((const char *)node->name, "device-problem") == 0)
			{
				xmlChar *problem_status = xmlNodeGetContent(node);
				if (problem_status)
				{
					if (strcmp((const char *)problem_status, "NO_PROBLEM") != 0)
					{
						fprintf(stderr, "Device problem detected: %s\n", (const char*)problem_status);
						xmlFree(problem_status);
						xmlFreeDoc(doc);
						return;
					}
					xmlFree(problem_status);
				}
			}
		}

		for (xmlNodePtr node = root_element->children; node != NULL; node = node->next)
		{
			if (node->type == XML_ELEMENT_NODE && strcmp((const char *)node->name, "device-service-infos") == 0)
			{
				for (xmlNodePtr info_node = node->children; info_node != NULL; info_node = info_node->next)
				{
					if (info_node->type == XML_ELEMENT_NODE && strcmp((const char *)info_node->name, "software-device-service-info") == 0)
					{
						for (xmlNodePtr child = info_node->children; child != NULL; child = child->next)
						{
							if (child->type == XML_ELEMENT_NODE)
							{
								if (strcmp((const char *)child->name, "product-name") == 0)
								{
									xmlChar *content = xmlNodeGetContent(child);
									if (content)
									{
										snprintf(current_product, sizeof(current_product), "%s", (const char*)content);
										xmlFree(content);
									}
								}
								else if (strcmp((const char *)child->name, "model-name") == 0)
								{
									xmlChar *content = xmlNodeGetContent(child);
									if (content)
									{
										snprintf(current_model, sizeof(current_model), "%s", (const char*)content);
										xmlFree(content);
									}
								}
								else if (strcmp((const char *)child->name, "name") == 0)
								{
									xmlChar *content = xmlNodeGetContent(child);
									if (content)
									{
										snprintf(current_name, sizeof(current_name), "%s", (const char*)content);
										xmlFree(content);
									}
								}
								else if (strcmp((const char *)child->name, "release-state") == 0)
								{
									xmlChar *content = xmlNodeGetContent(child);
									if (content)
									{
										snprintf(current_release_state, sizeof(current_release_state), "%s", (const char*)content);
										xmlFree(content);
									}
								}
								else if (strcmp((const char *)child->name, "link") == 0)
								{
									xmlChar *href = xmlGetProp(child, (const xmlChar *)"href");
									if (href)
									{
										snprintf(current_download_url, sizeof(current_download_url), "%s", (const char*)href);
										xmlFree(href);
									}
								}
							}
						}
					}
				}
			}
		}

		LOG("\n--- SONY DEVICE INFO ---\n");
		LOG("Product:      %s\n", current_product);
		LOG("Model:        %s\n", current_model);
		LOG("Full Name:    %s\n", current_name);
		LOG("Release State:%s\n", current_release_state);
		LOG("Download URL: %s\n", current_download_url);
		LOG("------------------------\n");

		xmlFreeDoc(doc);

		if (strlen(current_download_url) > 0)
		{
			struct MemoryStruct sub_chunk;
			sub_chunk.memory = malloc(1);
			sub_chunk.size = 0;

			if (download_xml(current_download_url, &sub_chunk))
			{
				parseWeatherXML(sub_chunk.memory);
			}
			else
			{
				fprintf(stderr, "Error downloading sub-XML file.\n");
			}

			free(sub_chunk.memory);
		}
		return;
	}
	else if (strcmp((const char *)root_element->name, "software-device-service") == 0)
	{
		parseSoftwareDetailsXML(root_element);
		xmlFreeDoc(doc);
		return;
	}

	xmlFreeDoc(doc);
}

int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		fprintf(stderr, "Usage: %s <enable_debug: 0 or 1> <url>\n", argv[0]);
		return EXIT_FAILURE;
	}

	// Ispravno cita indeks 1 i indeks 2 iz argv niza
	debug_enabled = atoi(argv[1]);
	const char *url = argv[2];

	curl_global_init(CURL_GLOBAL_ALL);
	CURL *curl_handle = curl_easy_init();
	CURLcode res;
	struct MemoryStruct chunk;

	chunk.memory = malloc(1); 
	chunk.size = 0; 

	if (curl_handle)
	{
		curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYPEER, 0L);
		curl_easy_setopt(curl_handle, CURLOPT_SSL_VERIFYHOST, 0L);
		curl_easy_setopt(curl_handle, CURLOPT_URL, url);
		curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
		curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&chunk);
		curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
		curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 5L);

		res = curl_easy_perform(curl_handle);

		if (res == CURLE_OK)
		{
			parseWeatherXML(chunk.memory);
		}
		else
		{
			fprintf(stderr, "CURL error: %s\n", curl_easy_strerror(res));
		}

		curl_easy_cleanup(curl_handle);
	}
	
	free(chunk.memory);
	curl_global_cleanup();
	xmlCleanupParser();
	
	return EXIT_SUCCESS;
}
