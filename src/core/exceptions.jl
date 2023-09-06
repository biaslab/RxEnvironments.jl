using Exceptions

mutable struct NotSubscribedException <: Exception
    origin::Any
    recipient::Any
end
origin(e::NotSubscribedException) = e.origin
recipient(e::NotSubscribedException) = e.recipient

Base.showerror(io::IO, e::NotSubscribedException) =
    print(io, "Entity $(origin(e)) is not subscribed to $(recipient(e))")


mutable struct MixedStateSpaceException <: Exception
    first::Any
    second::Any
end
first_entity(e::MixedStateSpaceException) = e.first
second_entity(e::MixedStateSpaceException) = e.second

Base.showerror(io::IO, e::MixedStateSpaceException) = print(
    io,
    "Entities $(first_entity(e)) and $(second_entity(e)) have different state spaces.",
)
