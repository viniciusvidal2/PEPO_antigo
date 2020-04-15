// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: arquivos.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_arquivos_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_arquivos_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3011000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3011004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_arquivos_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_arquivos_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_arquivos_2eproto;
namespace ArquivosMsgProto {
class Arquivos;
class ArquivosDefaultTypeInternal;
extern ArquivosDefaultTypeInternal _Arquivos_default_instance_;
}  // namespace ArquivosMsgProto
PROTOBUF_NAMESPACE_OPEN
template<> ::ArquivosMsgProto::Arquivos* Arena::CreateMaybeMessage<::ArquivosMsgProto::Arquivos>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ArquivosMsgProto {

// ===================================================================

class Arquivos :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ArquivosMsgProto.Arquivos) */ {
 public:
  Arquivos();
  virtual ~Arquivos();

  Arquivos(const Arquivos& from);
  Arquivos(Arquivos&& from) noexcept
    : Arquivos() {
    *this = ::std::move(from);
  }

  inline Arquivos& operator=(const Arquivos& from) {
    CopyFrom(from);
    return *this;
  }
  inline Arquivos& operator=(Arquivos&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Arquivos& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Arquivos* internal_default_instance() {
    return reinterpret_cast<const Arquivos*>(
               &_Arquivos_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Arquivos& a, Arquivos& b) {
    a.Swap(&b);
  }
  inline void Swap(Arquivos* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Arquivos* New() const final {
    return CreateMaybeMessage<Arquivos>(nullptr);
  }

  Arquivos* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Arquivos>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Arquivos& from);
  void MergeFrom(const Arquivos& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Arquivos* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ArquivosMsgProto.Arquivos";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_arquivos_2eproto);
    return ::descriptor_table_arquivos_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNuvensFieldNumber = 1,
    kImagensFieldNumber = 2,
  };
  // int32 nuvens = 1;
  void clear_nuvens();
  ::PROTOBUF_NAMESPACE_ID::int32 nuvens() const;
  void set_nuvens(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_nuvens() const;
  void _internal_set_nuvens(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // int32 imagens = 2;
  void clear_imagens();
  ::PROTOBUF_NAMESPACE_ID::int32 imagens() const;
  void set_imagens(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_imagens() const;
  void _internal_set_imagens(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:ArquivosMsgProto.Arquivos)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::int32 nuvens_;
  ::PROTOBUF_NAMESPACE_ID::int32 imagens_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_arquivos_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Arquivos

// int32 nuvens = 1;
inline void Arquivos::clear_nuvens() {
  nuvens_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Arquivos::_internal_nuvens() const {
  return nuvens_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Arquivos::nuvens() const {
  // @@protoc_insertion_point(field_get:ArquivosMsgProto.Arquivos.nuvens)
  return _internal_nuvens();
}
inline void Arquivos::_internal_set_nuvens(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  nuvens_ = value;
}
inline void Arquivos::set_nuvens(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_nuvens(value);
  // @@protoc_insertion_point(field_set:ArquivosMsgProto.Arquivos.nuvens)
}

// int32 imagens = 2;
inline void Arquivos::clear_imagens() {
  imagens_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Arquivos::_internal_imagens() const {
  return imagens_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Arquivos::imagens() const {
  // @@protoc_insertion_point(field_get:ArquivosMsgProto.Arquivos.imagens)
  return _internal_imagens();
}
inline void Arquivos::_internal_set_imagens(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  imagens_ = value;
}
inline void Arquivos::set_imagens(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_imagens(value);
  // @@protoc_insertion_point(field_set:ArquivosMsgProto.Arquivos.imagens)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace ArquivosMsgProto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_arquivos_2eproto
