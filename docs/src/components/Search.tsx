/// <reference types="user-agent-data-types" />

import { useEffect, useState } from "react";
import { SearchIcon } from "./SearchIcon";
import { useRef } from "react";

export const Search = () => {
  const dialogRef = useRef<HTMLDialogElement>(null);
  // const searchButtonRef = useRef<HTMLButtonElement>(null);

  const handleClick = () => {
    if (dialogRef.current) {
      dialogRef.current.showModal();
    }
  };

  // const blurSearchInput = () => {
  //   console.log("PRUTT");
  //   setTimeout(() => {
  //     if (searchButtonRef.current) {
  //       console.log("PRUTTIS");
  //       searchButtonRef.current.blur();
  //     }
  //   }, 10);
  // };

  return (
    <>
      <button
        aria-label="search"
        role="searchbox"
        // ref={searchButtonRef}
        // className="rounded-full border border-neutral-400 w-64 h-lg flex items-center pr-sm pl-md text-neutral-500 cursor-text"
        // className="btn btn-circle btn-ghost md:input md:input-bordered md:input-sm md:w-64 flex items-center md:cursor-text"
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
        <form
          method="dialog"
          className="modal-box absolute top-lg"
          // onBlur={blurSearchInput}
        >
          <input
            type="text"
            placeholder="Search here"
            className="input input-bordered w-full"
          />
          {/* <p className="py-4">
            Press ESC key or click the button below to close
          </p> */}
          <div className="modal-action">
            <button className="btn">Close</button>
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
