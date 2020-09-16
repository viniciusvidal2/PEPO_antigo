// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: arquivos.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "arquivos.pb.h"

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

namespace ArquivosMsgProto {
class ArquivosDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<Arquivos> {
} _Arquivos_default_instance_;

namespace protobuf_arquivos_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

const ::google::protobuf::uint32 TableStruct::offsets[] = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Arquivos, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Arquivos, pasta_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Arquivos, nuvens_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Arquivos, imagens_),
};

static const ::google::protobuf::internal::MigrationSchema schemas[] = {
  { 0, -1, sizeof(Arquivos)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_Arquivos_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "arquivos.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  _Arquivos_default_instance_.Shutdown();
  delete file_level_metadata[0].reflection;
}

void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _Arquivos_default_instance_.DefaultConstruct();
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] = {
      "\n\016arquivos.proto\022\020ArquivosMsgProto\":\n\010Ar"
      "quivos\022\r\n\005pasta\030\001 \001(\t\022\016\n\006nuvens\030\002 \001(\005\022\017\n"
      "\007imagens\030\003 \001(\005b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 102);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "arquivos.proto", &protobuf_RegisterTypes);
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

}  // namespace protobuf_arquivos_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Arquivos::kPastaFieldNumber;
const int Arquivos::kNuvensFieldNumber;
const int Arquivos::kImagensFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Arquivos::Arquivos()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_arquivos_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:ArquivosMsgProto.Arquivos)
}
Arquivos::Arquivos(const Arquivos& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  pasta_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.pasta().size() > 0) {
    pasta_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.pasta_);
  }
  ::memcpy(&nuvens_, &from.nuvens_,
    reinterpret_cast<char*>(&imagens_) -
    reinterpret_cast<char*>(&nuvens_) + sizeof(imagens_));
  // @@protoc_insertion_point(copy_constructor:ArquivosMsgProto.Arquivos)
}

void Arquivos::SharedCtor() {
  pasta_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&nuvens_, 0, reinterpret_cast<char*>(&imagens_) -
    reinterpret_cast<char*>(&nuvens_) + sizeof(imagens_));
  _cached_size_ = 0;
}

Arquivos::~Arquivos() {
  // @@protoc_insertion_point(destructor:ArquivosMsgProto.Arquivos)
  SharedDtor();
}

void Arquivos::SharedDtor() {
  pasta_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void Arquivos::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Arquivos::descriptor() {
  protobuf_arquivos_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_arquivos_2eproto::file_level_metadata[0].descriptor;
}

const Arquivos& Arquivos::default_instance() {
  protobuf_arquivos_2eproto::InitDefaults();
  return *internal_default_instance();
}

Arquivos* Arquivos::New(::google::protobuf::Arena* arena) const {
  Arquivos* n = new Arquivos;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Arquivos::Clear() {
// @@protoc_insertion_point(message_clear_start:ArquivosMsgProto.Arquivos)
  pasta_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&nuvens_, 0, reinterpret_cast<char*>(&imagens_) -
    reinterpret_cast<char*>(&nuvens_) + sizeof(imagens_));
}

bool Arquivos::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:ArquivosMsgProto.Arquivos)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // string pasta = 1;
      case 1: {
        if (tag == 10u) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_pasta()));
          DO_(::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
            this->pasta().data(), this->pasta().length(),
            ::google::protobuf::internal::WireFormatLite::PARSE,
            "ArquivosMsgProto.Arquivos.pasta"));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 nuvens = 2;
      case 2: {
        if (tag == 16u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &nuvens_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 imagens = 3;
      case 3: {
        if (tag == 24u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &imagens_)));
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
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:ArquivosMsgProto.Arquivos)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:ArquivosMsgProto.Arquivos)
  return false;
#undef DO_
}

void Arquivos::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:ArquivosMsgProto.Arquivos)
  // string pasta = 1;
  if (this->pasta().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
      this->pasta().data(), this->pasta().length(),
      ::google::protobuf::internal::WireFormatLite::SERIALIZE,
      "ArquivosMsgProto.Arquivos.pasta");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->pasta(), output);
  }

  // int32 nuvens = 2;
  if (this->nuvens() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->nuvens(), output);
  }

  // int32 imagens = 3;
  if (this->imagens() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->imagens(), output);
  }

  // @@protoc_insertion_point(serialize_end:ArquivosMsgProto.Arquivos)
}

::google::protobuf::uint8* Arquivos::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic;  // Unused
  // @@protoc_insertion_point(serialize_to_array_start:ArquivosMsgProto.Arquivos)
  // string pasta = 1;
  if (this->pasta().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
      this->pasta().data(), this->pasta().length(),
      ::google::protobuf::internal::WireFormatLite::SERIALIZE,
      "ArquivosMsgProto.Arquivos.pasta");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->pasta(), target);
  }

  // int32 nuvens = 2;
  if (this->nuvens() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->nuvens(), target);
  }

  // int32 imagens = 3;
  if (this->imagens() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->imagens(), target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:ArquivosMsgProto.Arquivos)
  return target;
}

size_t Arquivos::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ArquivosMsgProto.Arquivos)
  size_t total_size = 0;

  // string pasta = 1;
  if (this->pasta().size() > 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->pasta());
  }

  // int32 nuvens = 2;
  if (this->nuvens() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->nuvens());
  }

  // int32 imagens = 3;
  if (this->imagens() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->imagens());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Arquivos::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:ArquivosMsgProto.Arquivos)
  GOOGLE_DCHECK_NE(&from, this);
  const Arquivos* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Arquivos>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:ArquivosMsgProto.Arquivos)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:ArquivosMsgProto.Arquivos)
    MergeFrom(*source);
  }
}

void Arquivos::MergeFrom(const Arquivos& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ArquivosMsgProto.Arquivos)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.pasta().size() > 0) {

    pasta_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.pasta_);
  }
  if (from.nuvens() != 0) {
    set_nuvens(from.nuvens());
  }
  if (from.imagens() != 0) {
    set_imagens(from.imagens());
  }
}

void Arquivos::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:ArquivosMsgProto.Arquivos)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Arquivos::CopyFrom(const Arquivos& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ArquivosMsgProto.Arquivos)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Arquivos::IsInitialized() const {
  return true;
}

void Arquivos::Swap(Arquivos* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Arquivos::InternalSwap(Arquivos* other) {
  pasta_.Swap(&other->pasta_);
  std::swap(nuvens_, other->nuvens_);
  std::swap(imagens_, other->imagens_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Arquivos::GetMetadata() const {
  protobuf_arquivos_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_arquivos_2eproto::file_level_metadata[0];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Arquivos

// string pasta = 1;
void Arquivos::clear_pasta() {
  pasta_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
const ::std::string& Arquivos::pasta() const {
  // @@protoc_insertion_point(field_get:ArquivosMsgProto.Arquivos.pasta)
  return pasta_.GetNoArena();
}
void Arquivos::set_pasta(const ::std::string& value) {
  
  pasta_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ArquivosMsgProto.Arquivos.pasta)
}
#if LANG_CXX11
void Arquivos::set_pasta(::std::string&& value) {
  
  pasta_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:ArquivosMsgProto.Arquivos.pasta)
}
#endif
void Arquivos::set_pasta(const char* value) {
  
  pasta_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ArquivosMsgProto.Arquivos.pasta)
}
void Arquivos::set_pasta(const char* value, size_t size) {
  
  pasta_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ArquivosMsgProto.Arquivos.pasta)
}
::std::string* Arquivos::mutable_pasta() {
  
  // @@protoc_insertion_point(field_mutable:ArquivosMsgProto.Arquivos.pasta)
  return pasta_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
::std::string* Arquivos::release_pasta() {
  // @@protoc_insertion_point(field_release:ArquivosMsgProto.Arquivos.pasta)
  
  return pasta_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
void Arquivos::set_allocated_pasta(::std::string* pasta) {
  if (pasta != NULL) {
    
  } else {
    
  }
  pasta_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), pasta);
  // @@protoc_insertion_point(field_set_allocated:ArquivosMsgProto.Arquivos.pasta)
}

// int32 nuvens = 2;
void Arquivos::clear_nuvens() {
  nuvens_ = 0;
}
::google::protobuf::int32 Arquivos::nuvens() const {
  // @@protoc_insertion_point(field_get:ArquivosMsgProto.Arquivos.nuvens)
  return nuvens_;
}
void Arquivos::set_nuvens(::google::protobuf::int32 value) {
  
  nuvens_ = value;
  // @@protoc_insertion_point(field_set:ArquivosMsgProto.Arquivos.nuvens)
}

// int32 imagens = 3;
void Arquivos::clear_imagens() {
  imagens_ = 0;
}
::google::protobuf::int32 Arquivos::imagens() const {
  // @@protoc_insertion_point(field_get:ArquivosMsgProto.Arquivos.imagens)
  return imagens_;
}
void Arquivos::set_imagens(::google::protobuf::int32 value) {
  
  imagens_ = value;
  // @@protoc_insertion_point(field_set:ArquivosMsgProto.Arquivos.imagens)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace ArquivosMsgProto

// @@protoc_insertion_point(global_scope)