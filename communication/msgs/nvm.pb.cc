// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: nvm.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "nvm.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace NVMMsgProto {
class NVMDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<NVM> {
} _NVM_default_instance_;

namespace protobuf_nvm_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

const ::google::protobuf::uint32 TableStruct::offsets[] = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(NVM, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(NVM, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(NVM, name_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(NVM, nlinhas_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(NVM, linhas_),
  0,
  1,
  ~0u,
};

static const ::google::protobuf::internal::MigrationSchema schemas[] = {
  { 0, 7, sizeof(NVM)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_NVM_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "nvm.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace

void TableStruct::Shutdown() {
  _NVM_default_instance_.Shutdown();
  delete file_level_metadata[0].reflection;
}

void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _NVM_default_instance_.DefaultConstruct();
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] = {
      "\n\tnvm.proto\022\013NVMMsgProto\"4\n\003NVM\022\014\n\004name\030"
      "\001 \002(\t\022\017\n\007nlinhas\030\002 \002(\005\022\016\n\006linhas\030\003 \003(\t"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 78);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "nvm.proto", &protobuf_RegisterTypes);
  ::google::protobuf::internal::OnShutdown(&TableStruct::Shutdown);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_nvm_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int NVM::kNameFieldNumber;
const int NVM::kNlinhasFieldNumber;
const int NVM::kLinhasFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

NVM::NVM()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_nvm_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:NVMMsgProto.NVM)
}
NVM::NVM(const NVM& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0),
      linhas_(from.linhas_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_name()) {
    name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.name_);
  }
  nlinhas_ = from.nlinhas_;
  // @@protoc_insertion_point(copy_constructor:NVMMsgProto.NVM)
}

void NVM::SharedCtor() {
  _cached_size_ = 0;
  name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  nlinhas_ = 0;
}

NVM::~NVM() {
  // @@protoc_insertion_point(destructor:NVMMsgProto.NVM)
  SharedDtor();
}

void NVM::SharedDtor() {
  name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void NVM::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* NVM::descriptor() {
  protobuf_nvm_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_nvm_2eproto::file_level_metadata[0].descriptor;
}

const NVM& NVM::default_instance() {
  protobuf_nvm_2eproto::InitDefaults();
  return *internal_default_instance();
}

NVM* NVM::New(::google::protobuf::Arena* arena) const {
  NVM* n = new NVM;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void NVM::Clear() {
// @@protoc_insertion_point(message_clear_start:NVMMsgProto.NVM)
  linhas_.Clear();
  if (has_name()) {
    GOOGLE_DCHECK(!name_.IsDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited()));
    (*name_.UnsafeRawStringPointer())->clear();
  }
  nlinhas_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool NVM::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:NVMMsgProto.NVM)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string name = 1;
      case 1: {
        if (tag == 10u) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->name().data(), this->name().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "NVMMsgProto.NVM.name");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required int32 nlinhas = 2;
      case 2: {
        if (tag == 16u) {
          set_has_nlinhas();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &nlinhas_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated string linhas = 3;
      case 3: {
        if (tag == 26u) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->add_linhas()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->linhas(this->linhas_size() - 1).data(),
            this->linhas(this->linhas_size() - 1).length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "NVMMsgProto.NVM.linhas");
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:NVMMsgProto.NVM)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:NVMMsgProto.NVM)
  return false;
#undef DO_
}

void NVM::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:NVMMsgProto.NVM)
  // required string name = 1;
  if (has_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "NVMMsgProto.NVM.name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->name(), output);
  }

  // required int32 nlinhas = 2;
  if (has_nlinhas()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->nlinhas(), output);
  }

  // repeated string linhas = 3;
  for (int i = 0; i < this->linhas_size(); i++) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->linhas(i).data(), this->linhas(i).length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "NVMMsgProto.NVM.linhas");
    ::google::protobuf::internal::WireFormatLite::WriteString(
      3, this->linhas(i), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:NVMMsgProto.NVM)
}

::google::protobuf::uint8* NVM::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic;  // Unused
  // @@protoc_insertion_point(serialize_to_array_start:NVMMsgProto.NVM)
  // required string name = 1;
  if (has_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "NVMMsgProto.NVM.name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }

  // required int32 nlinhas = 2;
  if (has_nlinhas()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->nlinhas(), target);
  }

  // repeated string linhas = 3;
  for (int i = 0; i < this->linhas_size(); i++) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->linhas(i).data(), this->linhas(i).length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "NVMMsgProto.NVM.linhas");
    target = ::google::protobuf::internal::WireFormatLite::
      WriteStringToArray(3, this->linhas(i), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:NVMMsgProto.NVM)
  return target;
}

size_t NVM::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:NVMMsgProto.NVM)
  size_t total_size = 0;

  if (has_name()) {
    // required string name = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->name());
  }

  if (has_nlinhas()) {
    // required int32 nlinhas = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->nlinhas());
  }

  return total_size;
}
size_t NVM::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:NVMMsgProto.NVM)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required string name = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->name());

    // required int32 nlinhas = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->nlinhas());

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  // repeated string linhas = 3;
  total_size += 1 *
      ::google::protobuf::internal::FromIntSize(this->linhas_size());
  for (int i = 0; i < this->linhas_size(); i++) {
    total_size += ::google::protobuf::internal::WireFormatLite::StringSize(
      this->linhas(i));
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void NVM::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:NVMMsgProto.NVM)
  GOOGLE_DCHECK_NE(&from, this);
  const NVM* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const NVM>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:NVMMsgProto.NVM)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:NVMMsgProto.NVM)
    MergeFrom(*source);
  }
}

void NVM::MergeFrom(const NVM& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:NVMMsgProto.NVM)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  linhas_.MergeFrom(from.linhas_);
  if (from._has_bits_[0 / 32] & 3u) {
    if (from.has_name()) {
      set_has_name();
      name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.name_);
    }
    if (from.has_nlinhas()) {
      set_nlinhas(from.nlinhas());
    }
  }
}

void NVM::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:NVMMsgProto.NVM)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void NVM::CopyFrom(const NVM& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:NVMMsgProto.NVM)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool NVM::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  return true;
}

void NVM::Swap(NVM* other) {
  if (other == this) return;
  InternalSwap(other);
}
void NVM::InternalSwap(NVM* other) {
  linhas_.UnsafeArenaSwap(&other->linhas_);
  name_.Swap(&other->name_);
  std::swap(nlinhas_, other->nlinhas_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata NVM::GetMetadata() const {
  protobuf_nvm_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_nvm_2eproto::file_level_metadata[0];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// NVM

// required string name = 1;
bool NVM::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void NVM::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
void NVM::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
void NVM::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
const ::std::string& NVM::name() const {
  // @@protoc_insertion_point(field_get:NVMMsgProto.NVM.name)
  return name_.GetNoArena();
}
void NVM::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.name)
}
#if LANG_CXX11
void NVM::set_name(::std::string&& value) {
  set_has_name();
  name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:NVMMsgProto.NVM.name)
}
#endif
void NVM::set_name(const char* value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:NVMMsgProto.NVM.name)
}
void NVM::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:NVMMsgProto.NVM.name)
}
::std::string* NVM::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:NVMMsgProto.NVM.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
::std::string* NVM::release_name() {
  // @@protoc_insertion_point(field_release:NVMMsgProto.NVM.name)
  clear_has_name();
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
void NVM::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:NVMMsgProto.NVM.name)
}

// required int32 nlinhas = 2;
bool NVM::has_nlinhas() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void NVM::set_has_nlinhas() {
  _has_bits_[0] |= 0x00000002u;
}
void NVM::clear_has_nlinhas() {
  _has_bits_[0] &= ~0x00000002u;
}
void NVM::clear_nlinhas() {
  nlinhas_ = 0;
  clear_has_nlinhas();
}
::google::protobuf::int32 NVM::nlinhas() const {
  // @@protoc_insertion_point(field_get:NVMMsgProto.NVM.nlinhas)
  return nlinhas_;
}
void NVM::set_nlinhas(::google::protobuf::int32 value) {
  set_has_nlinhas();
  nlinhas_ = value;
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.nlinhas)
}

// repeated string linhas = 3;
int NVM::linhas_size() const {
  return linhas_.size();
}
void NVM::clear_linhas() {
  linhas_.Clear();
}
const ::std::string& NVM::linhas(int index) const {
  // @@protoc_insertion_point(field_get:NVMMsgProto.NVM.linhas)
  return linhas_.Get(index);
}
::std::string* NVM::mutable_linhas(int index) {
  // @@protoc_insertion_point(field_mutable:NVMMsgProto.NVM.linhas)
  return linhas_.Mutable(index);
}
void NVM::set_linhas(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.linhas)
  linhas_.Mutable(index)->assign(value);
}
void NVM::set_linhas(int index, const char* value) {
  linhas_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:NVMMsgProto.NVM.linhas)
}
void NVM::set_linhas(int index, const char* value, size_t size) {
  linhas_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:NVMMsgProto.NVM.linhas)
}
::std::string* NVM::add_linhas() {
  // @@protoc_insertion_point(field_add_mutable:NVMMsgProto.NVM.linhas)
  return linhas_.Add();
}
void NVM::add_linhas(const ::std::string& value) {
  linhas_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:NVMMsgProto.NVM.linhas)
}
void NVM::add_linhas(const char* value) {
  linhas_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:NVMMsgProto.NVM.linhas)
}
void NVM::add_linhas(const char* value, size_t size) {
  linhas_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:NVMMsgProto.NVM.linhas)
}
const ::google::protobuf::RepeatedPtrField< ::std::string>&
NVM::linhas() const {
  // @@protoc_insertion_point(field_list:NVMMsgProto.NVM.linhas)
  return linhas_;
}
::google::protobuf::RepeatedPtrField< ::std::string>*
NVM::mutable_linhas() {
  // @@protoc_insertion_point(field_mutable_list:NVMMsgProto.NVM.linhas)
  return &linhas_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace NVMMsgProto

// @@protoc_insertion_point(global_scope)
