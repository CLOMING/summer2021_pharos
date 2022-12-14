/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "PublicationData.h"

static int
datexPublish_SubscribeSerial_nbr_2_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

/*
 * This type is implemented using NativeInteger,
 * so here we adjust the DEF accordingly.
 */
static int
datexPublish_Serial_nbr_3_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

/*
 * This type is implemented using NativeInteger,
 * so here we adjust the DEF accordingly.
 */
static int
memb_datexPublish_SubscribeSerial_nbr_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

static int
memb_datexPublish_Serial_nbr_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

static asn_oer_constraints_t asn_OER_type_datexPublish_SubscribeSerial_nbr_constr_2 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_type_datexPublish_SubscribeSerial_nbr_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_datexPublish_Serial_nbr_constr_3 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_type_datexPublish_Serial_nbr_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_datexPublish_SubscribeSerial_nbr_constr_2 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_memb_datexPublish_SubscribeSerial_nbr_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_datexPublish_Serial_nbr_constr_3 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_memb_datexPublish_Serial_nbr_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_specifics_t asn_SPC_datexPublish_SubscribeSerial_nbr_specs_2 = {
	0,	0,	0,	0,	0,
	0,	/* Native long size */
	1	/* Unsigned representation */
};
static const ber_tlv_tag_t asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_datexPublish_SubscribeSerial_nbr_2 = {
	"datexPublish-SubscribeSerial-nbr",
	"datexPublish-SubscribeSerial-nbr",
	&asn_OP_NativeInteger,
	asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2,
	sizeof(asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2)
		/sizeof(asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2[0]) - 1, /* 1 */
	asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2,	/* Same as above */
	sizeof(asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2)
		/sizeof(asn_DEF_datexPublish_SubscribeSerial_nbr_tags_2[0]), /* 2 */
	{ &asn_OER_type_datexPublish_SubscribeSerial_nbr_constr_2, &asn_PER_type_datexPublish_SubscribeSerial_nbr_constr_2, datexPublish_SubscribeSerial_nbr_2_constraint },
	0, 0,	/* No members */
	&asn_SPC_datexPublish_SubscribeSerial_nbr_specs_2	/* Additional specs */
};

static const asn_INTEGER_specifics_t asn_SPC_datexPublish_Serial_nbr_specs_3 = {
	0,	0,	0,	0,	0,
	0,	/* Native long size */
	1	/* Unsigned representation */
};
static const ber_tlv_tag_t asn_DEF_datexPublish_Serial_nbr_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_datexPublish_Serial_nbr_3 = {
	"datexPublish-Serial-nbr",
	"datexPublish-Serial-nbr",
	&asn_OP_NativeInteger,
	asn_DEF_datexPublish_Serial_nbr_tags_3,
	sizeof(asn_DEF_datexPublish_Serial_nbr_tags_3)
		/sizeof(asn_DEF_datexPublish_Serial_nbr_tags_3[0]) - 1, /* 1 */
	asn_DEF_datexPublish_Serial_nbr_tags_3,	/* Same as above */
	sizeof(asn_DEF_datexPublish_Serial_nbr_tags_3)
		/sizeof(asn_DEF_datexPublish_Serial_nbr_tags_3[0]), /* 2 */
	{ &asn_OER_type_datexPublish_Serial_nbr_constr_3, &asn_PER_type_datexPublish_Serial_nbr_constr_3, datexPublish_Serial_nbr_3_constraint },
	0, 0,	/* No members */
	&asn_SPC_datexPublish_Serial_nbr_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_PublicationData_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PublicationData, datexPublish_SubscribeSerial_nbr),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_datexPublish_SubscribeSerial_nbr_2,
		0,
		{ &asn_OER_memb_datexPublish_SubscribeSerial_nbr_constr_2, &asn_PER_memb_datexPublish_SubscribeSerial_nbr_constr_2,  memb_datexPublish_SubscribeSerial_nbr_constraint_1 },
		0, 0, /* No default value */
		"datexPublish-SubscribeSerial-nbr"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PublicationData, datexPublish_Serial_nbr),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_datexPublish_Serial_nbr_3,
		0,
		{ &asn_OER_memb_datexPublish_Serial_nbr_constr_3, &asn_PER_memb_datexPublish_Serial_nbr_constr_3,  memb_datexPublish_Serial_nbr_constraint_1 },
		0, 0, /* No default value */
		"datexPublish-Serial-nbr"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PublicationData, datexPublish_LatePublicationFlag),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BOOLEAN,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datexPublish-LatePublicationFlag"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PublicationData, datexPublish_Type),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_PublicationType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datexPublish-Type"
		},
};
static const ber_tlv_tag_t asn_DEF_PublicationData_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PublicationData_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* datexPublish-SubscribeSerial-nbr */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* datexPublish-Serial-nbr */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* datexPublish-LatePublicationFlag */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* datexPublish-Type */
};
asn_SEQUENCE_specifics_t asn_SPC_PublicationData_specs_1 = {
	sizeof(struct PublicationData),
	offsetof(struct PublicationData, _asn_ctx),
	asn_MAP_PublicationData_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PublicationData = {
	"PublicationData",
	"PublicationData",
	&asn_OP_SEQUENCE,
	asn_DEF_PublicationData_tags_1,
	sizeof(asn_DEF_PublicationData_tags_1)
		/sizeof(asn_DEF_PublicationData_tags_1[0]), /* 1 */
	asn_DEF_PublicationData_tags_1,	/* Same as above */
	sizeof(asn_DEF_PublicationData_tags_1)
		/sizeof(asn_DEF_PublicationData_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_PublicationData_1,
	4,	/* Elements count */
	&asn_SPC_PublicationData_specs_1	/* Additional specs */
};

