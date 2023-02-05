
/**
 * @file
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>

#include <mqtt.h>

#include "templates/bearssl_sockets.h"

typedef struct {
    uint16_t data_length;
    uint16_t buffer_length;
    uint8_t *buffer;
    bool error;
} buffer_head;

int testCerts(br_x509_trust_anchor *anch);

/**
 * @brief The function that would be called whenever a PUBLISH is received.
 * 
 * @note This function is not used in this example. 
 */
static void publish_callback(void** unused, struct mqtt_response_publish *published);

/**
 * @brief Safely closes the socket in \p ctx before \c exit. 
 */
static void exit_example(int status, bearssl_context *ctx);

/* The next five functions decode a certificate into BearSSL format */

/**
 * @brief Callback function to accumulate data in a buffer
 */ 
static void vblob_append(void *cc, const void *data, size_t len);

/**
 * @brief Cleans up allocations made creating trusted anchors
 */
static void free_ta_contents(br_x509_trust_anchor *ta);

/**
 * @brief Converts certificate \p xc to a trust anchor in \ta (based on code in BearSSL tools)
 */
static int certificate_to_trust_anchor(br_x509_certificate *xc, br_x509_trust_anchor *ta);

/**
 * @brief Generates trust anchors for BearSSL from the contents of \p ca_file and stores them
 * in the \p anchoOut array (based on code in BearSSL tools)
 */ 
static size_t get_trusted_anchors_from_file(const char *ca_file, br_x509_trust_anchor **anchOut);
/**
 * @brief Generates trust anchors for BearSSL from the string \p ca and stores them
 * in the \p anchOut array (based on code in BearSSL tools)
 * 
 * @returns The number of trust anchors generated
 */ 
static size_t get_trusted_anchors(const unsigned char *ca, size_t ca_len, br_x509_trust_anchor **anchOut);

// Global to return Ctrl-C event
static volatile int stop = 0;

/**
 * @brief Tells main to quit loop and exit when a Ctrl-C is received
 */
void signalHandler(int signum) {
    (void)signum;
    stop = 1;
}

/**
 * A simple program to that publishes the current time until Ctrl-C is pressed. 
 */
int main(int argc, const char *argv[]) 
{
    const char* addr;
    const char* port;
    const char* topic;
    const char* ca_file;
    const char* subscribe;

    bearssl_context ctx;
    memset(&ctx, 0, sizeof(bearssl_context));

    if (argc > 1) {
        ca_file = argv[1];
    } else {
        ca_file = "./examples/mosquitto.org.pem";
    }

    /* get address (argv[2] if present) */
    if (argc > 2) {
        addr = argv[2];
    } else {
        addr = "test.mosquitto.org";
    }

    /* get port number (argv[3] if present) */
    if (argc > 3) {
        port = argv[3];
    } else {
        port = "8883";
    }

    /* get the topic name to publish */
    if (argc > 4) {
        topic = argv[4];
    } else {
        topic = "markrad/datetime";
    }

    if (argc > 5) {
        subscribe = argv[5];
    } else {
        subscribe = "markrad/#";
    }

    unsigned char bearssl_iobuf[BR_SSL_BUFSIZE_BIDI];

    /* generate BearSSL trusted anchors - specifically kept out of open_nb_socket since it needs to malloc */

    /* 
        Generate BearSSL trusted anchors 

        This code converts the certificate into a format that is readable by the BearSSL library. Sadly there isn't 
        a way to accomplish this without the use of malloc thus I specifically kept this code out of open_nb_socket.
        The author of the bearSSL library offers two options:

        1) Do the conversion of the certificate in your code. There are examples of how to do this. The benefit of 
           this is that you can run the same code against different servers by providing the appropriate trusted root
           pem file. The function get_trusted_anchors does exactly this.

        2) Use the tool provided with BearSSL to generate the C code that will initialize the trusted anchor structures.
           Essentially it simply generates initialized C structures that you can copy into your code. You will not need 
           to use malloc but you will lose some flexibility. For information on the tool see 
           this page: https://www.bearssl.org/api1.html
    */
    ctx.ta_count = get_trusted_anchors_from_file(ca_file, &ctx.anchOut);

    if (ctx.ta_count < 1) {
        fprintf(stderr, "Certificate file is invalid\n");
        exit_example(EXIT_FAILURE, &ctx);
    }

    /* deal with broken pipe in socket code */
    signal(SIGPIPE, SIG_IGN);

    /* open the non-blocking TCP socket (connecting to the broker) */
    if (0 != open_nb_socket(&ctx, addr, port, bearssl_iobuf, sizeof(bearssl_iobuf)))
    {
        fprintf(stderr, "Unable to open socket - %d\n", errno);
        exit_example(EXIT_FAILURE, &ctx);
    }

    /* setup a client */
    struct mqtt_client client;
    uint8_t sendbuf[2048]; /* sendbuf should be large enough to hold multiple whole mqtt messages */
    uint8_t recvbuf[1024]; /* recvbuf should be large enough any whole mqtt message expected to be received */
    mqtt_init(&client, &ctx, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf), publish_callback);
    mqtt_connect(&client, "publishing_client", NULL, NULL, 0, NULL, NULL, 0, 400);
    mqtt_subscribe(&client, subscribe, 0);

    /* check that we don't have any errors */
    if (client.error != MQTT_OK) {
        fprintf(stderr, "error: %s\n", mqtt_error_str(client.error));
        exit_example(EXIT_FAILURE, &ctx);
    }

    signal(SIGINT, signalHandler);

    /* start publishing the time */
    printf("%s is ready to begin publishing the time.\n", argv[0]);
    printf("Current time will be published every five seconds (or so).\n");
    printf("Press CTRL-C to exit.\n\n");

    int counter = 49;       /* 49 so it will publish the first time the loop is entered */
    int disconnect = 0;

    while(!stop) {
        if (++disconnect % 300 == 0)
        {
            printf("Reconnection test\n");
            mqtt_disconnect(&client);
            mqtt_sync(&client);

            /* check that we don't have any errors */
            if (client.error != MQTT_OK) {
                fprintf(stderr, "reconnection error: %s\n", mqtt_error_str(client.error));
                exit_example(EXIT_FAILURE, &ctx);
            }
            close_socket(&ctx);
                        
            if (0 != open_nb_socket(&ctx, addr, port, bearssl_iobuf, sizeof(bearssl_iobuf)))
            {
                fprintf(stderr, "Unable to open socket: %d\n", errno);
                exit_example(EXIT_FAILURE, &ctx);
            }

            if (MQTT_OK != mqtt_connect(&client, "publishing_client", NULL, NULL, 0, NULL, NULL, 0, 400)) {
                fprintf(stderr, "mqtt_connect failed: %s\n", mqtt_error_str(client.error));
                exit_example(EXIT_FAILURE, &ctx);
            }

            if (MQTT_OK != mqtt_subscribe(&client, subscribe, 0)) {
                fprintf(stderr, "mqtt_subscribe failed: %s\n", mqtt_error_str(client.error));
            }

            if (MQTT_OK != mqtt_sync(&client)) {
                fprintf(stderr, "Sync failed after reconnect: %s\n", mqtt_error_str(client.error));
                exit_example(EXIT_FAILURE, &ctx);
            }
        }
        if (++counter % 50 == 0) {
            counter = 0;
            /* get the current time */
            time_t timer;
            time(&timer);
            struct tm* tm_info = localtime(&timer);
            char timebuf[26];
            strftime(timebuf, 26, "%Y-%m-%d %H:%M:%S", tm_info);

            /* print a message */
            char application_message[256];
            snprintf(application_message, sizeof(application_message), "The time is %s", timebuf);

            /* publish the time */
            mqtt_publish(&client, topic, application_message, strlen(application_message) + 1, MQTT_PUBLISH_QOS_2);
            printf("%s published : \"%s\"\n", argv[0], application_message);

            /* check for errors */
            if (client.error != MQTT_OK) {
                fprintf(stderr, "error: %s\n", mqtt_error_str(client.error));
                exit_example(EXIT_FAILURE, &ctx);
            }
        }

        int rc;

        if (MQTT_OK != (rc = mqtt_sync(&client)))
        {
            printf("Communication failure: %d\n", rc);
            return 4;
        }
        usleep(100000U);
    }   

    /* disconnect */
    printf("\n%s disconnecting from %s\n", argv[0], addr);
    mqtt_disconnect(&client);
    mqtt_sync(&client);

    /* check that we don't have any errors */
    if (client.error != MQTT_OK) {
        fprintf(stderr, "error at termination: %s\n", mqtt_error_str(client.error));
        exit_example(EXIT_FAILURE, &ctx);
    }

    close_socket(&ctx);

    sleep(1);

    /* exit */ 
    exit_example(EXIT_SUCCESS, &ctx);
}

static void exit_example(int status, bearssl_context *ctx)
{
    close_socket(ctx);
    exit(status);
}

static void publish_callback(void** unused, struct mqtt_response_publish *published) 
{
    static const char *prelim = "Received publish('";
    /* note that published->topic_name is NOT null-terminated (here we'll change it to a c-string) */
    printf("%s", prelim);
    fwrite(published->topic_name, 1, published->topic_name_size, stdout);
    printf("'): %s\n", (const char*)published->application_message);
}

static void vblob_append(void *cc, const void *data, size_t len)
{
    buffer_head *bv = (buffer_head *)cc;

    if (bv->data_length + len > bv->buffer_length)
    {
        bv->buffer_length += 1024;                                  // Probably the most that will be allocated
        bv->buffer = realloc(bv->buffer, bv->buffer_length);        // We just have to hope this works
    }

    memcpy(bv->buffer + bv->data_length, data, len);
    bv->data_length += len;
}

static void free_ta_contents(br_x509_trust_anchor *ta)
{
	free(ta->dn.data);
	switch (ta->pkey.key_type) 
    {
	case BR_KEYTYPE_RSA:
		free(ta->pkey.key.rsa.n);
		free(ta->pkey.key.rsa.e);
		break;
	case BR_KEYTYPE_EC:
		free(ta->pkey.key.ec.q);
		break;
	}

    free(ta);
}

static int certificate_to_trust_anchor(br_x509_certificate *xc, br_x509_trust_anchor *ta) {

    // TODO: Review return value
	br_x509_decoder_context dc;
	br_x509_pkey *pk;
    buffer_head vdn;
    int result = 0;

    vdn.buffer = NULL;
    vdn.buffer_length = 0;
    vdn.data_length = 0;
    vdn.error = false;

    memset(ta, 0, sizeof(br_x509_trust_anchor));
    br_x509_decoder_init(&dc, vblob_append, &vdn);
    br_x509_decoder_push(&dc, xc->data, xc->data_len);
    pk = br_x509_decoder_get_pkey(&dc);

    if (pk == NULL) {
        return 0;
    }

    ta->dn.data = vdn.buffer;
    ta->dn.len = vdn.data_length;
    ta->flags = 0;

    if (br_x509_decoder_isCA(&dc)) 
    {
        ta->flags |= BR_X509_TA_CA;
    }

    switch (pk->key_type) {

    case BR_KEYTYPE_RSA:
        ta->pkey.key_type = BR_KEYTYPE_RSA;
        ta->pkey.key.rsa.nlen = pk->key.rsa.nlen;
        ta->pkey.key.rsa.elen = pk->key.rsa.elen;

        if (NULL == (ta->pkey.key.rsa.n = (unsigned char *)malloc(ta->pkey.key.rsa.nlen)) ||
            NULL == ( ta->pkey.key.rsa.e = (unsigned char *)malloc(ta->pkey.key.rsa.elen))) {
            free_ta_contents(ta);
            return 0;
        }
        else {
            memcpy(ta->pkey.key.rsa.n, pk->key.rsa.n, ta->pkey.key.rsa.nlen);
            memcpy(ta->pkey.key.rsa.e, pk->key.rsa.e, ta->pkey.key.rsa.elen);
            result = 0;
        }
        break;
    case BR_KEYTYPE_EC:
        ta->pkey.key_type = BR_KEYTYPE_EC;
        ta->pkey.key.ec.curve = pk->key.ec.curve;
        ta->pkey.key.ec.qlen = pk->key.ec.qlen;

        if (NULL == (ta->pkey.key.ec.q = (unsigned char *)malloc(ta->pkey.key.ec.qlen))) {
            free_ta_contents(ta);
            return 0;
        }
        else {
            memcpy(ta->pkey.key.ec.q, pk->key.ec.q, ta->pkey.key.ec.qlen);
            result = 0;
        }
        break;
    default:
        // ERROR: unsupported public key type in CA
        free_ta_contents(ta);
        return 0;
    }
}

static size_t get_trusted_anchors_from_file(const char *ca_file, br_x509_trust_anchor **anchOut)
{
    // Read the certificates from the file
    FILE *f = fopen(ca_file, "rb");
    char *certs = NULL;
    size_t rc = 0;

    if (ca_file != NULL) {
        if (f != NULL) {
            fseek(f, 0, SEEK_END);
            long fsize = ftell(f);
            fseek(f, 0, SEEK_SET);

            certs = malloc(fsize);

            if (certs != NULL) {
                size_t read = fread(certs, 1, fsize, f);
                
                fclose(f);

                if (read == fsize) {
                    rc = get_trusted_anchors(certs, read, anchOut);
                }
            }
        }
    }

    free(certs);

    return rc;
}

static size_t get_trusted_anchors(const unsigned char *ca, size_t ca_len, br_x509_trust_anchor **anchOut) {

    static const char CERTIFICATE[] = "CERTIFICATE";
    static const char X509_CERTIFICATE[] = "X509 CERTIFICATE";
    static const int CERTIFICATE_LEN = sizeof(CERTIFICATE) - 1;
    static const int X509_CERTIFICATE_LEN = sizeof(X509_CERTIFICATE) - 1;

    int cert_count = 0;
	const unsigned char *buf;
    buffer_head bv;
	char *po_name;
	size_t rc = 0;
  	br_pem_decoder_context pc;
    br_x509_certificate xc;
	int inobj;
	int extra_nl;
    bool error;
    br_x509_trust_anchor *tas = NULL;

    br_pem_decoder_init(&pc);
    buf = ca;
    inobj = 0;
	po_name;
	extra_nl = 1;
    error = false;

    // Decode the certificate string
    while (!error && ca_len > 0) {
		size_t tlen;

		tlen = br_pem_decoder_push(&pc, buf, ca_len);
		buf += tlen;
		ca_len -= tlen;
		switch (br_pem_decoder_event(&pc)) {

		case BR_PEM_BEGIN_OBJ:
            po_name = strdup(br_pem_decoder_name(&pc));
            if (po_name == NULL) {
                fprintf(stderr, "ERROR: out of memeory\n");
                error = true;
                continue;
            }
            bv.buffer = NULL;
            bv.buffer_length = 0;
            bv.data_length = 0;
            bv.error = false;
			br_pem_decoder_setdest(&pc, vblob_append, &bv);
			inobj = 1;
			break;

		case BR_PEM_END_OBJ:
			if (inobj) {
                if (bv.error == true) {
                    error = true;
                    continue;
                }
                if (0 == memcmp(po_name, CERTIFICATE, CERTIFICATE_LEN)
                || (0 == memcmp(po_name, X509_CERTIFICATE, X509_CERTIFICATE_LEN))) {

                    xc.data = bv.buffer;
                    bv.buffer = NULL;
                    xc.data_len = bv.data_length;
                    free(po_name);
                    po_name = NULL;
                    if (NULL == (tas = (br_x509_trust_anchor *)realloc(tas, (cert_count + 1) * sizeof(br_x509_trust_anchor)))) {
                		fprintf(stderr, "ERROR: out of memory\n");
                        error = true;
                        continue;
                    }
                    if (0 == certificate_to_trust_anchor(&xc, tas + cert_count)) {
                        error = true;
                        continue;
                    }
                    xc.data = NULL;
                    inobj = 0;
                    cert_count++;
                }
			}
			break;

		case BR_PEM_ERROR:
            error = true;
			continue;
		}

		/*
		 * We add an extra newline at the end, in order to
		 * support PEM files that lack the newline on their last
		 * line (this is somwehat invalid, but PEM format is not
		 * standardised and such files do exist in the wild, so
		 * we'd better accept them).
		 */
		if (ca_len == 0 && extra_nl) {
			extra_nl = 0;
			buf = (unsigned char *)"\n";
			ca_len = 1;
		}
	}

	if (inobj) {
		fprintf(stderr, "ERROR: unfinished PEM object\n");
        error = true;
    }

    if (error) {
		free(po_name);
		free(bv.buffer);
        free(xc.data);

        for (int i = 0; i < cert_count; i++) {
            free_ta_contents(*(anchOut + cert_count));
        }

		return 0;
	}

    *anchOut = tas;

	return cert_count;
}
