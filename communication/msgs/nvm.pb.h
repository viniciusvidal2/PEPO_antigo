// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: nvm.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_nvm_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_nvm_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_nvm_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_nvm_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_nvm_2eproto;
namespace NVMMsgProto {
class NVM;
class NVMDefaultTypeInternal;
extern NVMDefaultTypeInternal _NVM_default_instance_;
}  // namespace NVMMsgProto
PROTOBUF_NAMESPACE_OPEN
template<> ::NVMMsgProto::NVM* Arena::CreateMaybeMessage<::NVMMsgProto::NVM>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace NVMMsgProto {

// ===================================================================

class NVM :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:NVMMsgProto.NVM) */ {
 public:
  NVM();
  virtual ~NVM();

  NVM(const NVM& from);
  NVM(NVM&& from) noexcept
    : NVM() {
    *this = ::std::move(from);
  }

  inline NVM& operator=(const NVM& from) {
    CopyFrom(from);
    return *this;
  }
  inline NVM& operator=(NVM&& from) noexcept {
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
  static const NVM& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NVM* internal_default_instance() {
    return reinterpret_cast<const NVM*>(
               &_NVM_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(NVM& a, NVM& b) {
    a.Swap(&b);
  }
  inline void Swap(NVM* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline NVM* New() const final {
    return CreateMaybeMessage<NVM>(nullptr);
  }

  NVM* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<NVM>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const NVM& from);
  void MergeFrom(const NVM& from);
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
  void InternalSwap(NVM* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "NVMMsgProto.NVM";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_nvm_2eproto);
    return ::descriptor_table_nvm_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLinhasFieldNumber = 3,
    kNameFieldNumber = 1,
    kNlinhasFieldNumber = 2,
  };
  // repeated string linhas = 3;
  int linhas_size() const;
  private:
  int _internal_linhas_size() const;
  public:
  void clear_linhas();
  const std::string& linhas(int index) const;
  std::string* mutable_linhas(int index);
  void set_linhas(int index, const std::string& value);
  void set_linhas(int index, std::string&& value);
  void set_linhas(int index, const char* value);
  void set_linhas(int index, const char* value, size_t size);
  std::string* add_linhas();
  void add_linhas(const std::string& value);
  void add_linhas(std::string&& value);
  void add_linhas(const char* value);
  void add_linhas(const char* value, size_t size);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>& linhas() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>* mutable_linhas();
  private:
  const std::string& _internal_linhas(int index) const;
  std::string* _internal_add_linhas();
  public:

  // string name = 1;
  void clear_name();
  const std::string& name() const;
  void set_name(const std::string& value);
  void set_name(std::string&& value);
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  std::string* mutable_name();
  std::string* release_name();
  void set_allocated_name(std::string* name);
  private:
  const std::string& _internal_name() const;
  void _internal_set_name(const std::string& value);
  std::string* _internal_mutable_name();
  public:

  // int32 nlinhas = 2;
  void clear_nlinhas();
  ::PROTOBUF_NAMESPACE_ID::int32 nlinhas() const;
  void set_nlinhas(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_nlinhas() const;
  void _internal_set_nlinhas(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:NVMMsgProto.NVM)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string> linhas_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
  ::PROTOBUF_NAMESPACE_ID::int32 nlinhas_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_nvm_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// NVM

// string name = 1;
inline void NVM::clear_name() {
  name_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline const std::string& NVM::name() const {
  // @@protoc_insertion_point(field_get:NVMMsgProto.NVM.name)
  return _internal_name();
}
inline void NVM::set_name(const std::string& value) {
  _internal_set_name(value);
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.name)
}
inline std::string* NVM::mutable_name() {
  // @@protoc_insertion_point(field_mutable:NVMMsgProto.NVM.name)
  return _internal_mutable_name();
}
inline const std::string& NVM::_internal_name() const {
  return name_.GetNoArena();
}
inline void NVM::_internal_set_name(const std::string& value) {
  
  name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
}
inline void NVM::set_name(std::string&& value) {
  
  name_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:NVMMsgProto.NVM.name)
}
inline void NVM::set_name(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  
  name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:NVMMsgProto.NVM.name)
}
inline void NVM::set_name(const char* value, size_t size) {
  
  name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:NVMMsgProto.NVM.name)
}
inline std::string* NVM::_internal_mutable_name() {
  
  return name_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* NVM::release_name() {
  // @@protoc_insertion_point(field_release:NVMMsgProto.NVM.name)
  
  return name_.ReleaseNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void NVM::set_allocated_name(std::string* name) {
  if (name != nullptr) {
    
  } else {
    
  }
  name_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:NVMMsgProto.NVM.name)
}

// int32 nlinhas = 2;
inline void NVM::clear_nlinhas() {
  nlinhas_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 NVM::_internal_nlinhas() const {
  return nlinhas_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 NVM::nlinhas() const {
  // @@protoc_insertion_point(field_get:NVMMsgProto.NVM.nlinhas)
  return _internal_nlinhas();
}
inline void NVM::_internal_set_nlinhas(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  nlinhas_ = value;
}
inline void NVM::set_nlinhas(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_nlinhas(value);
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.nlinhas)
}

// repeated string linhas = 3;
inline int NVM::_internal_linhas_size() const {
  return linhas_.size();
}
inline int NVM::linhas_size() const {
  return _internal_linhas_size();
}
inline void NVM::clear_linhas() {
  linhas_.Clear();
}
inline std::string* NVM::add_linhas() {
  // @@protoc_insertion_point(field_add_mutable:NVMMsgProto.NVM.linhas)
  return _internal_add_linhas();
}
inline const std::string& NVM::_internal_linhas(int index) const {
  return linhas_.Get(index);
}
inline const std::string& NVM::linhas(int index) const {
  // @@protoc_insertion_point(field_get:NVMMsgProto.NVM.linhas)
  return _internal_linhas(index);
}
inline std::string* NVM::mutable_linhas(int index) {
  // @@protoc_insertion_point(field_mutable:NVMMsgProto.NVM.linhas)
  return linhas_.Mutable(index);
}
inline void NVM::set_linhas(int index, const std::string& value) {
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.linhas)
  linhas_.Mutable(index)->assign(value);
}
inline void NVM::set_linhas(int index, std::string&& value) {
  // @@protoc_insertion_point(field_set:NVMMsgProto.NVM.linhas)
  linhas_.Mutable(index)->assign(std::move(value));
}
inline void NVM::set_linhas(int index, const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  linhas_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:NVMMsgProto.NVM.linhas)
}
inline void NVM::set_linhas(int index, const char* value, size_t size) {
  linhas_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:NVMMsgProto.NVM.linhas)
}
inline std::string* NVM::_internal_add_linhas() {
  return linhas_.Add();
}
inline void NVM::add_linhas(const std::string& value) {
  linhas_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:NVMMsgProto.NVM.linhas)
}
inline void NVM::add_linhas(std::string&& value) {
  linhas_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:NVMMsgProto.NVM.linhas)
}
inline void NVM::add_linhas(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  linhas_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:NVMMsgProto.NVM.linhas)
}
inline void NVM::add_linhas(const char* value, size_t size) {
  linhas_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:NVMMsgProto.NVM.linhas)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>&
NVM::linhas() const {
  // @@protoc_insertion_point(field_list:NVMMsgProto.NVM.linhas)
  return linhas_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>*
NVM::mutable_linhas() {
  // @@protoc_insertion_point(field_mutable_list:NVMMsgProto.NVM.linhas)
  return &linhas_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace NVMMsgProto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_nvm_2eproto
