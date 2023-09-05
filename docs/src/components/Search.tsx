import { useState } from "react";
import { SearchIcon } from "./SearchIcon";
import { useRef } from "react";
import Fuse from "fuse.js";
import type { MarkdownHeading } from "astro";
import { XIcon } from "./XIcon";

export type DocCategory = "article" | "news post";

export interface SearchItem {
  title: string;
  slug: string;
  fullPath: string;
  headings?: MarkdownHeading[];
  category: DocCategory;
  body: string;
}

export const Search = ({ items }: { items: SearchItem[] }) => {
  const [searchString, setSearchString] = useState("");
  const [searchResults, setSearchResults] = useState<
    Fuse.FuseResult<SearchItem>[]
  >([]);
  const dialogRef = useRef<HTMLDialogElement>(null);

  const handleClick = () => {
    if (dialogRef.current) {
      dialogRef.current.showModal();
    }
  };

  const fuse = new Fuse(items, {
    keys: [
      { name: "title", weight: 5 },
      { name: "headings.text", weight: 1 },
      { name: "body", weight: 0.5 },
    ],
    includeMatches: true,
    includeScore: true,
    minMatchCharLength: 2,
    threshold: 0.2,
    ignoreLocation: true,
  });

  const doSearch = (input: string) => {
    const res = fuse.search(input);
    setSearchResults(res);
  };

  const trackSearch = (val: string) => {
    // @ts-expect-error - gtag is global
    gtag("event", "search", {
      search_term: val,
    });
  };

  const handleBlur = () => {
    trackSearch(searchString);
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setSearchString(e.target.value);
    doSearch(e.target.value);
  };

  const handleClearClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    setSearchString("");
    doSearch("");
  };

  return (
    <>
      <button
        aria-label="search"
        role="searchbox"
        className="search-input-button"
        onClick={handleClick}
      >
        <div className="flex justify-between md:grow items-center">
          <span className="text-neutral-400 hidden md:inline-block">
            Search
          </span>
          <span>
            <SearchIcon />
          </span>
        </div>
      </button>
      <dialog id="search-modal" className={`modal`} ref={dialogRef}>
        <form method="dialog" className="modal-box absolute top-lg">
          <div className="relative flex">
            <input
              className="input input-bordered grow"
              placeholder="Search here"
              value={searchString}
              onChange={handleInputChange}
              onBlur={handleBlur}
            />
            <button
              className="btn btn-ghost btn-sm btn-circle absolute right-1 top-2"
              onClick={handleClearClick}
            >
              <XIcon />
            </button>
          </div>
          <div className="flex flex-col w-full mt-md gap-sm">
            {searchResults.slice(0, 10).map((sr) => (
              <SearchResult key={sr.item.slug} res={sr} />
            ))}
          </div>
          <div className="modal-action">
            <button className="btn btn-ghost">Close</button>
          </div>
        </form>
        <form
          method="dialog"
          className="modal-backdrop bg-neutral-700 opacity-20"
        >
          <button>close</button>
        </form>
      </dialog>
    </>
  );
};

const SearchResult = ({ res }: { res: Fuse.FuseResult<SearchItem> }) => {
  return (
    <>
      <div className="grid card border border-neutral-400 rounded-box p-md">
        <div className="flex items-start">
          <div className="grow mr-xs">
            <a className="link link-primary font-bold" href={res.item.fullPath}>
              {res.item.title}
            </a>
          </div>
          <div className="mt-[2px]">
            <CategoryBadge category={res.item.category} />
          </div>
        </div>
        <Headings
          fullPath={res.item.fullPath}
          headings={res.item.headings || []}
        />
      </div>
    </>
  );
};

const CategoryBadge = ({ category }: { category: DocCategory }) => {
  return (
    <div className="badge badge-secondary badge-outline capitalize">
      {category}
    </div>
  );
};

const Headings = ({
  headings,
  fullPath,
}: {
  fullPath: string;
  headings: MarkdownHeading[];
}) => {
  const headingsWithoutFirstH1 = headings.filter(
    (h, index) => h.depth !== 1 && index !== 0
  );
  return (
    <div>
      {headingsWithoutFirstH1.map((h) => {
        return (
          <a
            className="block link link-secondary"
            href={`${fullPath}#${h.slug}`}
            key={h.slug}
          >
            {h.text}
          </a>
        );
      })}
    </div>
  );
};
