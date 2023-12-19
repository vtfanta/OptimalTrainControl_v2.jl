using OptimalTrainControl_v2
using Documenter

DocMeta.setdocmeta!(OptimalTrainControl_v2, :DocTestSetup, :(using OptimalTrainControl_v2); recursive=true)

makedocs(;
    modules=[OptimalTrainControl_v2],
    authors="Vít Fanta <fantavit@fel.cvut.cz> and contributors",
    repo="https://github.com/vtfanta/OptimalTrainControl_v2.jl/blob/{commit}{path}#{line}",
    sitename="OptimalTrainControl_v2.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        edit_link="master",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)
